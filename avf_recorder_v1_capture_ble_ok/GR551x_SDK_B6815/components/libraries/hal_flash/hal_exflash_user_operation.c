#include "hal_exflash_user_operation.h"

/* NOTE : Here is a function demo if user want to realize XIP Flash operations.
          Please refer hal_flash_read_identification_id()
 */
#define FLASH_MANU_ID_INVALID0    0x00
#define FLASH_MANU_ID_INVALID1    0xFF

extern exflash_handle_t g_exflash_handle;
extern hal_status_t hal_xqspi_command_receive_patch(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry);

static uint32_t s_identification_id;
static SECTION_RAM_CODE hal_status_t exflash_read_identification_id(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;
    xqspi_command_t command;
    uint8_t id[3];

    command.inst = SPI_FLASH_CMD_RDID;
    command.addr = 0;
    command.inst_size = XQSPI_INSTSIZE_08_BITS;
    command.addr_size = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode = XQSPI_DATA_MODE_SPI;
    command.length = 3;

    status = hal_xqspi_command_receive_patch(p_exflash->p_xqspi, &command, id, 1000);
    if (HAL_OK != status)
    {
        return status;
    }

    if ((FLASH_MANU_ID_INVALID0 != id[0]) && (FLASH_MANU_ID_INVALID1 != id[0]))
    {
        s_identification_id = id[2] + (id[1] << 8) + (id[0] << 16);
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

uint32_t hal_flash_read_identification_id(void)
{
    hal_exflash_operation(&g_exflash_handle, exflash_read_identification_id);
    return s_identification_id;
}
