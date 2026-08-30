Here is the Markdown table matching the ADS1299 register map table:

| ADDRESS | REGISTER | DEFAULT SETTING | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Read Only ID Registers** |  |  |  |  |  |  |  |  |  |  |
| 00h | ID | exh | REV_ID[2:0] | REV_ID[2:0] | REV_ID[2:0] | 1 | DEV_ID[1:0] | DEV_ID[1:0] | NU_CH[1:0] | NU_CH[1:0] |
| **Global Settings Across Channels** |  |  |  |  |  |  |  |  |  |  |
| 01h | CONFIG1 | 96h | 1 | $\overline{\text{DAISY\_EN}}$ | CLK_EN | 1 | 0 | DR[2:0] | DR[2:0] | DR[2:0] |
| 02h | CONFIG2 | C0h | 1 | 1 | 0 | INT_CAL | 0 | CAL_AMP0 | CAL_FREQ[1:0] | CAL_FREQ[1:0] |
| 03h | CONFIG3 | 60h | PD_REFBUF | 1 | 1 | BIAS_MEAS | BIASREF_INT | $\overline{\text{PD\_BIAS}}$ | BIAS_LOFF_SENS | BIAS_STAT |
| 04h | LOFF | 00h | COMP_TH[2:0] | COMP_TH[2:0] | COMP_TH[2:0] | 0 | ILEAD_OFF[1:0] | ILEAD_OFF[1:0] | FLEAD_OFF[1:0] | FLEAD_OFF[1:0] |
| **Channel-Specific Settings** |  |  |  |  |  |  |  |  |  |  |
| 05h | CH1SET | 61h | PD1 | GAIN1[2:0] | GAIN1[2:0] | GAIN1[2:0] | SRB2 | MUX1[2:0] | MUX1[2:0] | MUX1[2:0] |
| 06h | CH2SET | 61h | PD2 | GAIN2[2:0] | GAIN2[2:0] | GAIN2[2:0] | SRB2 | MUX2[2:0] | MUX2[2:0] | MUX2[2:0] |
| 07h | CH3SET | 61h | PD3 | GAIN3[2:0] | GAIN3[2:0] | GAIN3[2:0] | SRB2 | MUX3[2:0] | MUX3[2:0] | MUX3[2:0] |
| 08h | CH4SET | 61h | PD4 | GAIN4[2:0] | GAIN4[2:0] | GAIN4[2:0] | SRB2 | MUX4[2:0] | MUX4[2:0] | MUX4[2:0] |
| 09h | CH5SET | 61h | PD5 | GAIN5[2:0] | GAIN5[2:0] | GAIN5[2:0] | SRB2 | MUX5[2:0] | MUX5[2:0] | MUX5[2:0] |
| 0Ah | CH6SET | 61h | PD6 | GAIN6[2:0] | GAIN6[2:0] | GAIN6[2:0] | SRB2 | MUX6[2:0] | MUX6[2:0] | MUX6[2:0] |
| 0Bh | CH7SET | 61h | PD7 | GAIN7[2:0] | GAIN7[2:0] | GAIN7[2:0] | SRB2 | MUX7[2:0] | MUX7[2:0] | MUX7[2:0] |
| 0Ch | CH8SET | 61h | PD8 | GAIN8[2:0] | GAIN8[2:0] | GAIN8[2:0] | SRB2 | MUX8[2:0] | MUX8[2:0] | MUX8[2:0] |
| 0Dh | BIAS_SENSP | 00h | BIASP8 | BIASP7 | BIASP6 | BIASP5 | BIASP4 | BIASP3 | BIASP2 | BIASP1 |
| 0Eh | BIAS_SENSN | 00h | BIASN8 | BIASN7 | BIASN6 | BIASN5 | BIASN4 | BIASN3 | BIASN2 | BIASN1 |
| 0Fh | LOFF_SENSP | 00h | LOFFP8 | LOFFP7 | LOFFP6 | LOFFP5 | LOFFP4 | LOFFP3 | LOFFP2 | LOFFP1 |
| 10h | LOFF_SENSN | 00h | LOFFM8 | LOFFM7 | LOFFM6 | LOFFM5 | LOFFM4 | LOFFM3 | LOFFM2 | LOFFM1 |
| 11h | LOFF_FLIP | 00h | LOFF_FLIP8 | LOFF_FLIP7 | LOFF_FLIP6 | LOFF_FLIP5 | LOFF_FLIP4 | LOFF_FLIP3 | LOFF_FLIP2 | LOFF_FLIP1 |
| **Lead-Off Status Registers (Read-Only Registers)** |  |  |  |  |  |  |  |  |  |  |
| 12h | LOFF_STATP | 00h | IN8P_OFF | IN7P_OFF | IN6P_OFF | IN5P_OFF | IN4P_OFF | IN3P_OFF | IN2P_OFF | IN1P_OFF |
| 13h | LOFF_STATN | 00h | IN8M_OFF | IN7M_OFF | IN6M_OFF | IN5M_OFF | IN4M_OFF | IN3M_OFF | IN2M_OFF | IN1M_OFF |
| **GPIO and OTHER Registers** |  |  |  |  |  |  |  |  |  |  |
| 14h | GPIO | 0Fh | GPIOD[4:1] | GPIOD[4:1] | GPIOD[4:1] | GPIOD[4:1] | GPIOC[4:1] | GPIOC[4:1] | GPIOC[4:1] | GPIOC[4:1] |
| 15h | MISC1 | 00h | 0 | 0 | SRB1 | 0 | 0 | 0 | 0 | 0 |
| 16h | MISC2 | 00h | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 17h | CONFIG4 | 00h | 0 | 0 | 0 | 0 | SINGLE_SHOT | 0 | $\overline{\text{PD\_LOFF\_COMP}}$ | 0 |