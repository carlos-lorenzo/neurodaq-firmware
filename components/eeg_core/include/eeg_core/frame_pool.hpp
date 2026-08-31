#pragma once

#include <cstdint>
#include <atomic>
#include <cstddef>
#include <array>
#include <utility>
#include <memory>


namespace eeg {
    /**
     * @brief A lock-free, Single-Producer Multi-Consumer (SPMC) reference-counted frame pool.
     *        Designed and optimized to avoid CPU false-sharing and memory fragmentation.
     *
     * @tparam T The type of data stored inside each frame slot.
     * @tparam Capacity Total number of frames available in the pool.
     * @tparam Consumers Expected number of consumer threads reading each frame.
     */
    template <typename T, std::size_t Capacity, std::size_t Consumers>
    class alignas(64) FramePool {
        private:
        struct FrameSlot {
            T data;
            std::atomic<std::size_t> ref_count{Consumers};
        };

        public:
        /**
         * @brief Construct a new Frame Pool and initialize the internal free list tracking chain.
         */
        FramePool() noexcept {
            for (std::size_t i = 0; i < Capacity; ++i) {
                free_list_[i].store(i + 1, std::memory_order_relaxed);
            }
        }

        // Disable copying and assignment due to atomic and unique ownership structures
        FramePool(const FramePool&) = delete;
        FramePool& operator=(const FramePool&) = delete;
        FramePool(FramePool&&) noexcept = delete;
        FramePool& operator=(FramePool&&) noexcept = delete;

        ~FramePool() noexcept = default;

        /**
         * @brief Allocates an open frame slot and constructs object T in-place.
         * @note Only safe to call from the designated SINGLE producer thread.
         *
         * @param args Forwarded arguments for T's constructor.
         * @return T* Pointer to the constructed payload, or nullptr if the pool is full.
         */
        template <typename... Args>
[[nodiscard]] T* allocate(Args&&... args) noexcept
        {
            std::size_t index =
                free_head_.load(std::memory_order_acquire);

            while (index < Capacity) {
                const std::size_t next =
                    free_list_[index].load(std::memory_order_acquire);

                if (free_head_.compare_exchange_weak(
                        index,
                        next,
                        std::memory_order_acquire,
                        std::memory_order_relaxed)) {
                    break;
                        }
            }

            if (index >= Capacity) {
                return nullptr;
            }

            FrameSlot& slot = slots_[index];

            std::construct_at(
                &slot.data,
                std::forward<Args>(args)...
            );

            slot.ref_count.store(
                Consumers,
                std::memory_order_release
            );

            free_count_.fetch_sub(1, std::memory_order_relaxed);

            return &slot.data;
        }

        /**
         * @brief Decrements a frame's reference count. Automatically cleans and recycles when 0.
         * @note Safe to execute concurrently from MULTIPLE consumer tracking execution threads.
         *
         * @param data_ptr Pointer to the initialized data member object to release.
         */
        void release(T* data_ptr) noexcept {
            if (!data_ptr) return;

            // 1. Reverse offset pointer arithmetic to map payload address to parent FrameSlot instance
            auto* slot_ptr = reinterpret_cast<FrameSlot*>(
                reinterpret_cast<char*>(data_ptr) - offsetof(FrameSlot, data)
            );
            std::size_t index = slot_ptr - slots_.data();

            // 2. Atomically drop down references. acq_rel enforces execution bounds sequencing.
            if (slot_ptr->ref_count.fetch_sub(1, std::memory_order_acq_rel) == 1) {

                // 3. This executing consumer dropped the final reference: trigger object destruction.
                std::destroy_at(&slot_ptr->data);

                // 4. Recycle slot index safely right back into the free collection tracking head via CAS
                std::size_t current_head = free_head_.load(std::memory_order_relaxed);
                while (true) {
                    free_list_[index].store(current_head, std::memory_order_relaxed);

                    if (free_head_.compare_exchange_weak(current_head, index,
                                                         std::memory_order_release,
                                                         std::memory_order_relaxed)) {
                        break;
                                                         }
                }

                // 5. Re-increment tracked free inventory space
                free_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }

        /**
         * @brief Access the approximate number of completely free slots available.
         */
        [[nodiscard]] std::size_t free_count() const noexcept {
            return free_count_.load(std::memory_order_relaxed);
        }

        private:
        std::array<FrameSlot, Capacity> slots_;
        std::array<std::atomic<std::size_t>, Capacity> free_list_;

        std::atomic<std::size_t> free_head_{0};
        std::atomic<std::size_t> free_count_{Capacity};
    };
}