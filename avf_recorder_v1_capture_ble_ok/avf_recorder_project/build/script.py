import sys
import re

file_path = r'f:\GR551x\GR551x_V1_6_10_B6815_项目交接\projects\ble\ble_peripheral\ble_app_uart_inmp441_wm8960\Src\audio_loopback\wm8960_inmp441.c'
with open(file_path, 'r', encoding='utf-8') as f:
    s = f.read()

s = s.replace('wm8960_', 'wm8962_')
s = s.replace('WM8960_', 'WM8962_')

pattern = r'static bool wm8962_write_reg\(.*?(return \(ret == APP_DRV_SUCCESS\);|return ret == APP_DRV_SUCCESS;)\s*\}'
new_fn = '''static bool wm8962_write_reg(uint16_t reg, uint16_t val)
{
    uint8_t data[4];
    data[0] = (reg >> 8) & 0xFF; // addr MSB
    data[1] = reg & 0xFF;        // addr LSB
    data[2] = (val >> 8) & 0xFF; // data MSB
    data[3] = val & 0xFF;        // data LSB
    
    uint16_t ret = app_i2c_transmit_sync(APP_I2C_ID_1, g_wm8962_addr, data, 4, 100);
    return (ret == APP_DRV_SUCCESS);
}'''
s = re.sub(pattern, new_fn, s, flags=re.DOTALL)

s = s.replace('wm8962_write_reg_checked(uint8_t reg,', 'wm8962_write_reg_checked(uint16_t reg,')

init_pattern = r'static bool wm8962_init\(void\).*?return true;\s*\}'
new_init = '''static bool wm8962_init(void)
{
    if (!wm8962_write_reg_checked(0x000F, 0x0000, "RESET")) return false;
    delay_ms(20);

    /* 电源与通路初始化 */
    if (!wm8962_write_reg_checked(0x0019, 0x004A, "PWR1")) return false;
    delay_ms(50);
    if (!wm8962_write_reg_checked(0x001A, 0x01E0, "PWR2")) return false;

    /* I2S 从机，16-bit，I2S格式 */
    if (!wm8962_write_reg_checked(0x0005, 0x0002, "AIF0")) return false;

    /* 取消 DAC 静音 */
    if (!wm8962_write_reg_checked(0x000A, 0x0000, "DAC_MUTE")) return false;

    /* 耳机输出音量 */
    if (!wm8962_write_reg_checked(0x0002, 0x0139, "HPOUTL_VOL")) return false;
    if (!wm8962_write_reg_checked(0x0003, 0x0139, "HPOUTR_VOL")) return false;

    /* 将 DAC 混到耳机输出 */
    if (!wm8962_write_reg_checked(0x0020, 0x0001, "LDAC_MIX")) return false;
    if (!wm8962_write_reg_checked(0x0025, 0x0001, "RDAC_MIX")) return false;

    return true;
}'''
s = re.sub(init_pattern, new_init, s, flags=re.DOTALL)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(s)
print('Done!')
