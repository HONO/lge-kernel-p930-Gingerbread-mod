#include <linux/module.h>
#include <lg_diagcmd.h>
#include <linux/input.h>
#include <linux/syscalls.h>

#include <lg_fw_diag_communication.h>
#include <lg_diag_testmode.h>
#include <mach/qdsp5v2/audio_def.h>
#include <linux/delay.h>

#ifndef SKW_TEST
#include <linux/fcntl.h> 
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif

#ifdef CONFIG_LGE_DLOAD_SRD
#include <userDataBackUpDiag.h>
#include <userDataBackUpTypeDef.h> 
#include <../../arch/arm/mach-msm/smd_private.h>
#include <linux/slab.h>
#endif 

#include <mach/board_lge.h>
#include <lg_backup_items.h>

#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <mach/irqs.h>

#define PMIC_GPIO_SDC3_DET 22
#define PM8058_GPIO_BASE NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio) (pm_gpio + PM8058_GPIO_BASE)

static struct diagcmd_dev *diagpdev;

extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern PACK(void *) diagpkt_free (PACK(void *)pkt);
extern void send_to_arm9( void * pReq, void * pRsp);
extern testmode_user_table_entry_type testmode_mstr_tbl[TESTMODE_MSTR_TBL_SIZE];
extern int diag_event_log_start(void);
extern int diag_event_log_end(void);
extern void set_operation_mode(boolean isOnline);
extern struct input_dev* get_ats_input_dev(void);

#ifdef CONFIG_LGE_DIAG_KEYPRESS
extern unsigned int LGF_KeycodeTrans(word input);
extern void LGF_SendKey(word keycode);
#endif

extern int boot_info;
extern int testmode_result;

extern void remote_rpc_srd_cmmand(void * pReq, void * pRsp );
extern void *smem_alloc(unsigned id, unsigned size);


extern PACK (void *)LGE_Dload_SRD (PACK (void *)req_pkt_ptr, uint16 pkg_len);
extern void diag_SRD_Init(udbp_req_type * req_pkt, udbp_rsp_type * rsp_pkt);
extern void diag_userDataBackUp_entrySet(udbp_req_type * req_pkt, udbp_rsp_type * rsp_pkt, script_process_type MODEM_MDM );
extern boolean writeBackUpNVdata( char * ram_start_address , unsigned int size);
extern void diag_userDataBackUp_data(udbp_req_type *req_pkt, udbp_rsp_type *rsp_pkt);	//CSFB SRD
extern boolean srd_userDataRecovery(void);	//CSFB SRD

#ifdef CONFIG_LGE_DLOAD_SRD  //kabjoo.choi
#define SIZE_OF_SHARD_RAM  0x60000  //384K

extern int lge_erase_block(int secnum, size_t size);
extern int lge_write_block(int secnum, unsigned char *buf, size_t size);
extern int lge_read_block(int secnum, unsigned char *buf, size_t size);
extern int lge_mmc_scan_partitions(void);

extern unsigned int srd_bytes_pos_in_emmc ;
unsigned char * load_srd_base;	//CSFB SRD
//CSFB SRD	unsigned char * load_srd_shard_base;
//CSFB SRD	unsigned char * load_srd_kernel_base;
#endif 

/* ==========================================================================
===========================================================================*/

struct statfs_local {
 __u32 f_type;
 __u32 f_bsize;
 __u32 f_blocks;
 __u32 f_bfree;
 __u32 f_bavail;
 __u32 f_files;
 __u32 f_ffree;
 __kernel_fsid_t f_fsid;
 __u32 f_namelen;
 __u32 f_frsize;
 __u32 f_spare[5];
};

/* ==========================================================================
===========================================================================*/

extern int get_touch_ts_fw_version(char *fw_ver);
extern int lge_bd_rev;

void CheckHWRev(byte *pStr)
{
    char *rev_str[] = {"evb1", "evb2", "A", "B", "C", "D",
        "E", "F", "G", "1.0", "1.1", "1.2",
        "revserved"};

    strcpy((char *)pStr ,(char *)rev_str[lge_bd_rev]);
}

PACK (void *)LGF_TestMode (
        PACK (void	*)req_pkt_ptr, /* pointer to request packet */
        uint16 pkt_len )        /* length of request packet */
{
    DIAG_TEST_MODE_F_req_type *req_ptr = (DIAG_TEST_MODE_F_req_type *) req_pkt_ptr;
    DIAG_TEST_MODE_F_rsp_type *rsp_ptr;
    unsigned int rsp_len=0;
    testmode_func_type func_ptr= NULL;
    int nIndex = 0;

    diagpdev = diagcmd_get_dev();

    // DIAG_TEST_MODE_F_rsp_type union type is greater than the actual size, decrease it in case sensitive items
    switch(req_ptr->sub_cmd_code)
    {
        case TEST_MODE_FACTORY_RESET_CHECK_TEST:
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
            break;

        case TEST_MODE_TEST_SCRIPT_MODE:
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_test_script_mode_type);
            break;

        //REMOVE UNNECESSARY RESPONSE PACKET FOR EXTERNEL SOCKET ERASE
        case TEST_MODE_EXT_SOCKET_TEST:
            if((req_ptr->test_mode_req.esm == EXTERNAL_SOCKET_ERASE) || (req_ptr->test_mode_req.esm == EXTERNAL_SOCKET_ERASE_SDCARD_ONLY) \
                    || (req_ptr->test_mode_req.esm == EXTERNAL_SOCKET_ERASE_FAT_ONLY))
                rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
            else
                rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
            break;

        //Added by jaeopark 110527 for XO Cal Backup
        case TEST_MODE_XO_CAL_DATA_COPY:
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_XOCalDataBackup_Type);
            break;

        case TEST_MODE_MANUAL_TEST_MODE:
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_manual_test_mode_type);
            break;

        case TEST_MODE_BLUETOOTH_RW:
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_bt_addr_type);
            break;

        case TEST_MODE_WIFI_MAC_RW:
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_wifi_addr_type);
            break;

        default :
            rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
            break;
    }

    rsp_ptr = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);

    printk(KERN_ERR "[LGF_TestMode] rsp_len: %d, sub_cmd_code: %d \n", rsp_len, req_ptr->sub_cmd_code);

    if (!rsp_ptr)
        return 0;

    rsp_ptr->sub_cmd_code = req_ptr->sub_cmd_code;
    rsp_ptr->ret_stat_code = TEST_OK_S; // test ok

    for( nIndex = 0 ; nIndex < TESTMODE_MSTR_TBL_SIZE  ; nIndex++)
    {
        if( testmode_mstr_tbl[nIndex].cmd_code == req_ptr->sub_cmd_code)
        {
            if( testmode_mstr_tbl[nIndex].which_procesor == ARM11_PROCESSOR)
                func_ptr = testmode_mstr_tbl[nIndex].func_ptr;
            break;
        }
    }

    if( func_ptr != NULL)
        return func_ptr( &(req_ptr->test_mode_req), rsp_ptr);
    else
    {
        if(req_ptr->test_mode_req.version == VER_HW)
            CheckHWRev((byte *)rsp_ptr->test_mode_rsp.str_buf);
        else if(req_ptr->test_mode_req.version == VER_TOUCH_FW)
            get_touch_ts_fw_version((byte *)rsp_ptr->test_mode_rsp.str_buf);
        else
            send_to_arm9((void*)req_ptr, (void*)rsp_ptr);
    }

    return (rsp_ptr);
}
EXPORT_SYMBOL(LGF_TestMode);

void* linux_app_handler(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    diagpkt_free(pRsp);
    return 0;
}

void* not_supported_command_handler(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
    return pRsp;
}

#define GET_INODE_FROM_FILEP(filp) \
    (filp)->f_path.dentry->d_inode

static int android_readwrite_file(const char *filename, char *rbuf, const char *wbuf, size_t length)
{
    int ret = 0;
    struct file *filp = (struct file *)-ENOENT;
    mm_segment_t oldfs;
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    do {
        int mode = (wbuf) ? O_RDWR : O_RDONLY;
        filp = filp_open(filename, mode, S_IRUSR);
        if (IS_ERR(filp) || !filp->f_op) {
            printk(KERN_ERR "%s: file %s filp_open error\n", __FUNCTION__, filename);
            ret = -ENOENT;
            break;
        }

        if (length==0) {
            /* Read the length of the file only */
            struct inode    *inode;

            inode = GET_INODE_FROM_FILEP(filp);
            if (!inode) {
                printk(KERN_ERR "%s: Get inode from %s failed\n", __FUNCTION__, filename);
                ret = -ENOENT;
                break;
            }
            ret = i_size_read(inode->i_mapping->host);
            break;
        }

        if (wbuf) {
            if ( (ret=filp->f_op->write(filp, wbuf, length, &filp->f_pos)) < 0) {
                printk(KERN_ERR "%s: Write %u bytes to file %s error %d\n", __FUNCTION__, 
                                length, filename, ret);
                break;
            }
        } else {
            if ( (ret=filp->f_op->read(filp, rbuf, length, &filp->f_pos)) < 0) {
                printk(KERN_ERR "%s: Read %u bytes from file %s error %d\n", __FUNCTION__,
                                length, filename, ret);
                break;
            }
        }
    } while (0);

    if (!IS_ERR(filp)) {
        filp_close(filp, NULL);
    }
    set_fs(oldfs);

    return ret;
}

char external_memory_copy_test(void)
{
    char return_value = TEST_FAIL_S;
    char *src = (void *)0;
    char *dest = (void *)0;
    off_t fd_offset;
    int fd;
    mm_segment_t old_fs=get_fs();
    set_fs(get_ds());

    if ( (fd = sys_open((const char __user *) "/sdcard/SDTest.txt", O_CREAT | O_RDWR, 0) ) < 0 )
    {
        printk(KERN_ERR "[Testmode Memory Test] Can not access SD card\n");
        goto file_fail;
    }

    if ( (src = kmalloc(10, GFP_KERNEL)) )
    {
        sprintf(src,"TEST");
        if ((sys_write(fd, (const char __user *) src, 5)) < 0)
        {
            printk(KERN_ERR "[Testmode Memory Test] Can not write SD card \n");
            goto file_fail;
        }

        fd_offset = sys_lseek(fd, 0, 0);
    }

    if ( (dest = kmalloc(10, GFP_KERNEL)) )
    {
        if ((sys_read(fd, (char __user *) dest, 5)) < 0)
        {
            printk(KERN_ERR "[Testmode Memory Test] Can not read SD card \n");
            goto file_fail;
        }

        if ((memcmp(src, dest, 4)) == 0)
            return_value = TEST_OK_S;
        else
            return_value = TEST_FAIL_S;
    }

    kfree(src);
    kfree(dest);

file_fail:
    sys_close(fd);
    set_fs(old_fs);
    sys_unlink((const char __user *)"/sdcard/SDTest.txt");

    return return_value;
}

extern int external_memory_test;

void* LGF_ExternalSocketMemory(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    int i;
    struct statfs_local sf;
    pRsp->ret_stat_code = TEST_FAIL_S;

    // ADD: 0013541: 0014142: [Test_Mode] To remove Internal memory information in External memory test when SD-card is not exist
    //if(gpio_get_value(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1)))
    if(external_memory_copy_test())
    {
        if (pReq->esm == EXTERNAL_SOCKET_MEMORY_CHECK)
        {
            pRsp->test_mode_rsp.memory_check = TEST_FAIL_S;
            pRsp->ret_stat_code = TEST_OK_S;
        }
        
        printk(KERN_ERR "[Testmode Memory Test] Can not detect SD card\n");
        return pRsp;
    }

    switch( pReq->esm){
        case EXTERNAL_SOCKET_MEMORY_CHECK:
            pRsp->test_mode_rsp.memory_check = external_memory_copy_test();
            pRsp->ret_stat_code = TEST_OK_S;
            break;

        case EXTERNAL_FLASH_MEMORY_SIZE:
            if (sys_statfs("/sdcard", (struct statfs *)&sf) != 0)
            {
                printk(KERN_ERR "[Testmode Memory Test] can not get sdcard infomation \n");
                pRsp->ret_stat_code = TEST_FAIL_S;
                break;
            }

            pRsp->test_mode_rsp.socket_memory_size = ((long long)sf.f_blocks * (long long)sf.f_bsize) >> 20; // needs Mb.
            pRsp->ret_stat_code = TEST_OK_S;
            break;

        case EXTERNAL_SOCKET_ERASE:
            testmode_result = -1;

            if (diagpdev != NULL)
            {
                update_diagcmd_state(diagpdev, "MMCFORMAT", 1);
            }
            else
            {
                printk("\n[%s] error EXTERNAL_SOCKET_ERASE", __func__ );
                pRsp->ret_stat_code = TEST_FAIL_S;
                break;
            }

            for (i =0; i < 20; i++)
            {
                if (testmode_result !=-1)
                    break;

                msleep(500);
            }

            if(testmode_result != -1)
            {
                pRsp->ret_stat_code = TEST_OK_S;
            }
            else
            {
                pRsp->ret_stat_code = TEST_FAIL_S;
                printk(KERN_ERR "[MMCFORMAT] DiagCommandObserver returned fail or didn't return in 10000ms.\n");
            }

            break;

        case EXTERNAL_FLASH_MEMORY_USED_SIZE:
            external_memory_test = -1;

            if (diagpdev != NULL)
            {
                update_diagcmd_state(diagpdev, "CALCUSEDSIZE", 0);
            }
            else
            {
                printk("\n[%s] error EXTERNAL_FLASH_MEMORY_USED_SIZE", __func__ );
                pRsp->ret_stat_code = TEST_FAIL_S;
                break;
            }

            for (i =0; i < 10; i++)
            {
                if (external_memory_test !=-1)
                    break;

                msleep(200);
            }

            if(external_memory_test != -1)
            {
                pRsp->test_mode_rsp.socket_memory_usedsize = external_memory_test;
                pRsp->ret_stat_code = TEST_OK_S;
            }
            else
            {
                pRsp->ret_stat_code = TEST_FAIL_S;
                printk(KERN_ERR "[CALCUSEDSIZE] DiagCommandObserver returned fail or didn't return in 2000ms.\n");
            }

            break;

        case EXTERNAL_FLASH_MEMORY_CONTENTS_CHECK:
            external_memory_test = -1;

            if (diagpdev != NULL)
            {
                update_diagcmd_state(diagpdev, "CHECKCONTENTS", 0);
            }
            else
            {
                printk("\n[%s] error EXTERNAL_FLASH_MEMORY_CONTENTS_CHECK", __func__ );
                pRsp->ret_stat_code = TEST_FAIL_S;
                break;
            }

            for (i =0; i < 10; i++)
            {
                if (external_memory_test !=-1)
                    break;

                msleep(200);
            }

            if(external_memory_test != -1)
            {
                if(external_memory_test == 1)
                    pRsp->test_mode_rsp.memory_check = TEST_OK_S;
                else 
                    pRsp->test_mode_rsp.memory_check = TEST_FAIL_S;

                pRsp->ret_stat_code = TEST_OK_S;
            }
            else
            {
                pRsp->ret_stat_code = TEST_FAIL_S;
                printk(KERN_ERR "[CHECKCONTENTS] DiagCommandObserver returned fail or didn't return in 2000ms.\n");
            }
            
            break;

        case EXTERNAL_FLASH_MEMORY_ERASE:
            external_memory_test = -1;

            if (diagpdev != NULL)
            {
                update_diagcmd_state(diagpdev, "ERASEMEMORY", 0);
            }
            else
            {
                printk("\n[%s] error EXTERNAL_FLASH_MEMORY_ERASE", __func__ );
                pRsp->ret_stat_code = TEST_FAIL_S;
                break;
            }


            for (i =0; i < 10; i++)
            {
                if (external_memory_test !=-1)
                    break;

                msleep(500);
            }

            if(external_memory_test != -1)
            {
                if(external_memory_test == 1)
                    pRsp->test_mode_rsp.memory_check = TEST_OK_S;
                else
                    pRsp->test_mode_rsp.memory_check = TEST_FAIL_S;

                pRsp->ret_stat_code = TEST_OK_S;
            }
            else
            {
                pRsp->ret_stat_code = TEST_FAIL_S;
                printk(KERN_ERR "[ERASEMEMORY] DiagCommandObserver returned fail or didn't return in 5000ms.\n");
            }
            
            break;

        case EXTERNAL_SOCKET_ERASE_SDCARD_ONLY: /*0xE*/
            if (diagpdev != NULL)
            {
                update_diagcmd_state(diagpdev, "MMCFORMAT", EXTERNAL_SOCKET_ERASE_SDCARD_ONLY);
                msleep(5000);
                pRsp->ret_stat_code = TEST_OK_S;
            }
            else
            {
                printk("\n[%s] error EXTERNAL_SOCKET_ERASE_SDCARD_ONLY", __func__ );
                pRsp->ret_stat_code = TEST_FAIL_S;
            }
            break;

        case EXTERNAL_SOCKET_ERASE_FAT_ONLY: /*0xF*/
            if (diagpdev != NULL)
            {
                update_diagcmd_state(diagpdev, "MMCFORMAT", EXTERNAL_SOCKET_ERASE_FAT_ONLY);
                msleep(5000);
                pRsp->ret_stat_code = TEST_OK_S;
            }
            else
            {
                printk("\n[%s] error EXTERNAL_SOCKET_ERASE_FAT_ONLY", __func__ );
                pRsp->ret_stat_code = TEST_FAIL_S;
            }
            break;

        default:
            pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
            break;
    }

    return pRsp;
}

void* LGF_TestModeBattLevel(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
#ifdef CONFIG_LGE_BATT_SOC_FOR_NPST
    int battery_soc = 0;
    extern int max17040_get_battery_capacity_percent(void);

    pRsp->ret_stat_code = TEST_OK_S;

    printk(KERN_ERR "%s, pRsp->ret_stat_code : %d\n", __func__, pReq->batt);
    if(pReq->batt == BATTERY_FUEL_GAUGE_SOC_NPST)
    {
        battery_soc = (int)max17040_get_battery_capacity_percent();
    }
    else
    {
        pRsp->ret_stat_code = TEST_FAIL_S;
    }

    if(battery_soc > 100)
        battery_soc = 100;
    else if (battery_soc < 0)
        battery_soc = 0;

    printk(KERN_ERR "%s, battery_soc : %d\n", __func__, battery_soc);

    sprintf((char *)pRsp->test_mode_rsp.batt_voltage, "%d", battery_soc);

    printk(KERN_ERR "%s, battery_soc : %s\n", __func__, (char *)pRsp->test_mode_rsp.batt_voltage);
#endif

    return pRsp;
}

void* LGF_TestModeKeyData(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{

    pRsp->ret_stat_code = TEST_OK_S;

#ifdef CONFIG_LGE_DIAG_KEYPRESS
    LGF_SendKey(LGF_KeycodeTrans(pReq->key_data));
#endif

    return pRsp;
}

extern struct device *get_atcmd_dev(void);

void* LGF_TestModeSleepMode(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    char *envp[3];
    char *atcmd_name = "AT_NAME=AT%FLIGHT";
    char *atcmd_state = "AT_STATE==1";
    struct device * dev = NULL;

    pRsp->ret_stat_code = TEST_FAIL_S;

    switch(pReq->sleep_mode)
    {
        case SLEEP_MODE_ON:
            dev = get_atcmd_dev();

            if (dev)
            {
                envp[0] = atcmd_name;
                envp[1] = atcmd_state;
                envp[2] = NULL;

                kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
                pRsp->ret_stat_code = TEST_OK_S;
            }
            break;

        case AIR_PLAIN_MODE_ON:
        case FTM_BOOT_ON:
            pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
            break;

        default:
            pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
            break;
    }

    return pRsp;
}

void* LGF_TestModeVirtualSimTest(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    pRsp->ret_stat_code = TEST_OK_S;
    return pRsp;
}

void* LGF_TestModeFBoot(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    switch( pReq->fboot)
    {
        case FIRST_BOOTING_COMPLETE_CHECK:
            if (boot_info)
                pRsp->ret_stat_code = TEST_OK_S;
            else
                pRsp->ret_stat_code = TEST_FAIL_S;

            break;

        default:
            pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
            break;
    }

    return pRsp;
}

extern int db_integrity_ready;
extern int fpri_crc_ready;
extern int file_crc_ready;
extern int db_dump_ready;
extern int db_copy_ready;

typedef struct {
    char ret[32];
} testmode_rsp_from_diag_type;

extern testmode_rsp_from_diag_type integrity_ret;
void* LGF_TestModeDBIntegrityCheck(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    int i;
    unsigned int crc_val;

    memset(integrity_ret.ret, 0, 32);

    if (diagpdev != NULL)
    {
        db_integrity_ready = 0;
        fpri_crc_ready = 0;
        file_crc_ready = 0;
        db_dump_ready = 0;
        db_copy_ready = 0;

        update_diagcmd_state(diagpdev, "DBCHECK", pReq->db_check);

        switch(pReq->db_check)
        {
            case DB_INTEGRITY_CHECK:
                for (i =0; i < 10; i++)
                {
                    if (db_integrity_ready)
                        break;

                    msleep(500);
                }

                msleep(500); // wait until the return value is written to the file

                crc_val = (unsigned int)simple_strtoul(integrity_ret.ret+1,NULL,16);
                sprintf(pRsp->test_mode_rsp.str_buf, "0x%08X", crc_val);

                printk(KERN_INFO "%s\n", integrity_ret.ret);
                printk(KERN_INFO "%d\n", crc_val);
                printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);

                pRsp->ret_stat_code = TEST_OK_S;
                break;

            case FPRI_CRC_CHECK:
                for (i =0; i < 10; i++)
                {
                    if (fpri_crc_ready)
                        break;

                    msleep(500);
                }

                msleep(500); // wait until the return value is written to the file

                crc_val = (unsigned int)simple_strtoul(integrity_ret.ret+1,NULL,16);
                sprintf(pRsp->test_mode_rsp.str_buf, "0x%08X", crc_val);

                printk(KERN_INFO "%s\n", integrity_ret.ret);
                printk(KERN_INFO "%d\n", crc_val);
                printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);

                pRsp->ret_stat_code = TEST_OK_S;
                break;

            case FILE_CRC_CHECK:
                for (i =0; i < 20; i++)
                {
                    if (file_crc_ready)
                        break;

                    msleep(500);
                }

                msleep(500); // wait until the return value is written to the file

                crc_val = (unsigned int)simple_strtoul(integrity_ret.ret+1,NULL,16);
                sprintf(pRsp->test_mode_rsp.str_buf, "0x%08X", crc_val);

                printk(KERN_INFO "%s\n", integrity_ret.ret);
                printk(KERN_INFO "%d\n", crc_val);
                printk(KERN_INFO "%s\n", pRsp->test_mode_rsp.str_buf);

                pRsp->ret_stat_code = TEST_OK_S;
                break;

            case CODE_PARTITION_CRC_CHECK:
                pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
                break;

            case TOTAL_CRC_CHECK:
                pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
                break;

            case DB_DUMP_CHECK:
                for (i =0; i < 10; i++)
                {
                    if (db_dump_ready)
                        break;

                    msleep(500);
                }

                msleep(500); // wait until the return value is written to the file

                if (integrity_ret.ret[0] == '0')
                    pRsp->ret_stat_code = TEST_OK_S;
                else
                    pRsp->ret_stat_code = TEST_FAIL_S;

                break;

            case DB_COPY_CHECK:
                for (i =0; i < 10; i++)
                {
                    if (db_copy_ready)
                        break;

                    msleep(500);
                }

                msleep(500); // wait until the return value is written to the file

                if (integrity_ret.ret[0] == '0')
                    pRsp->ret_stat_code = TEST_OK_S;
                else
                    pRsp->ret_stat_code = TEST_FAIL_S;

                break;

            default :
                pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
                break;
        }
    }
    else
    {
        printk("\n[%s] error DBCHECK", __func__ );
        pRsp->ret_stat_code = TEST_FAIL_S;
    }

    printk(KERN_ERR "[_DBCHECK_] [%s:%d] DBCHECK Result=<%s>\n", __func__, __LINE__, integrity_ret.ret);

    return pRsp;
}

extern byte fota_id_read[20];

void* LGF_TestModeFOTA(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    int i;
    
    if (diagpdev != NULL)
    {
        switch( pReq->fota)
        {
            case FOTA_ID_READ:
                for(i=0; i<19; i++)
                    fota_id_read[i] = 0;

                update_diagcmd_state(diagpdev, "FOTAIDREAD", 0);
                msleep(500);

                for(i=0; i<19; i++)
                    pRsp->test_mode_rsp.fota_id_read[i] = fota_id_read[i];

                printk(KERN_ERR "%s, rsp.read_fota_id : %s\n", __func__, (char *)pRsp->test_mode_rsp.fota_id_read);
                pRsp->ret_stat_code = TEST_OK_S;
                break;

            default:
                pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
                break;
        }
    }
    else
        pRsp->ret_stat_code = TEST_FAIL_S;

    return pRsp;
}


// LGE_CHANGE_S, bill.jung@lge.com, 20110808, WiFi MAC R/W Function by DIAG
void* LGF_TestModeWiFiMACRW(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
	int fd=0; 
	int i=0;
    char *src = (void *)0;	
    mm_segment_t old_fs=get_fs();
    set_fs(get_ds());

	printk(KERN_ERR "[LGF_TestModeWiFiMACRW] req_type=%d, wifi_mac_addr=[%s]\n", pReq->wifi_mac_ad.req_type, pReq->wifi_mac_ad.wifi_mac_addr);

	if (diagpdev != NULL)
	{
		if( pReq->wifi_mac_ad.req_type == 0 )
		{
			printk(KERN_ERR "[LGF_TestModeWiFiMACRW] WIFI_MAC_ADDRESS_WRITE.\n");
			
			if ( (fd = sys_open((const char __user *) "/data/misc/wifi/diag_mac", O_CREAT | O_RDWR, 0777) ) < 0 )
		    {
		    	printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not open file.\n");
				pRsp->ret_stat_code = TEST_FAIL_S;
				goto file_fail;
		    }
				
			if ( (src = kmalloc(20, GFP_KERNEL)) )
			{
				sprintf( src,"%c%c%c%c%c%c%c%c%c%c%c%c", pReq->wifi_mac_ad.wifi_mac_addr[0],
					pReq->wifi_mac_ad.wifi_mac_addr[1], pReq->wifi_mac_ad.wifi_mac_addr[2],
					pReq->wifi_mac_ad.wifi_mac_addr[3], pReq->wifi_mac_ad.wifi_mac_addr[4],
					pReq->wifi_mac_ad.wifi_mac_addr[5], pReq->wifi_mac_ad.wifi_mac_addr[6],
					pReq->wifi_mac_ad.wifi_mac_addr[7], pReq->wifi_mac_ad.wifi_mac_addr[8],
					pReq->wifi_mac_ad.wifi_mac_addr[9], pReq->wifi_mac_ad.wifi_mac_addr[10],
					pReq->wifi_mac_ad.wifi_mac_addr[11]
					);
					
				if ((sys_write(fd, (const char __user *) src, 12)) < 0)
				{
					printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not write file.\n");
					pRsp->ret_stat_code = TEST_FAIL_S;
					goto file_fail;
				}
			}

			msleep(500);
				
			update_diagcmd_state(diagpdev, "WIFIMACWRITE", 0);
				
			pRsp->ret_stat_code = TEST_OK_S;

		}
		else if(  pReq->wifi_mac_ad.req_type == 1 )
		{
			printk(KERN_ERR "[LGF_TestModeWiFiMACRW] WIFI_MAC_ADDRESS_READ.\n");
			
			update_diagcmd_state(diagpdev, "WIFIMACREAD", 0);

			for( i=0; i< 2; i++ )
			{
				msleep(500);
			}					

			if ( (fd = sys_open((const char __user *) "/data/misc/wifi/diag_mac", O_CREAT | O_RDWR, 0777) ) < 0 )
		    {
		    	printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not open file.\n");
				pRsp->ret_stat_code = TEST_FAIL_S;
				goto file_fail;
		    }
			
			if ( (src = kmalloc(20, GFP_KERNEL)) )
			{
				if ((sys_read(fd, (char __user *) src, 12)) < 0)
				{
					printk(KERN_ERR "[LGF_TestModeWiFiMACRW] Can not read file.\n");
					pRsp->ret_stat_code = TEST_FAIL_S;
					goto file_fail;
				}
			}

			for( i=0; i<14; i++)
			{
				pRsp->test_mode_rsp.key_pressed_buf[i] = 0;
			}

			for( i=0; i< 12; i++ )
			{
				pRsp->test_mode_rsp.read_wifi_mac_addr[i] = src[i];
			}

			sys_unlink((const char __user *)"/data/misc/wifi/diag_mac");
					
			pRsp->ret_stat_code = TEST_OK_S;
		}				
		else
		{
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
	}
	else
	{
		pRsp->ret_stat_code = TEST_FAIL_S;
	}

file_fail:
	kfree(src);
	
	sys_close(fd);
	set_fs(old_fs); 
	
	return pRsp;
}
// LGE_CHANGE_E, bill.jung@lge.com, 20110808, WiFi MAC R/W Function by DIAG

void* LGF_TestModePowerReset(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    if (diagpdev != NULL)
    {
        update_diagcmd_state(diagpdev, "REBOOT", 0);
        pRsp->ret_stat_code = TEST_OK_S;
    }
    else
        pRsp->ret_stat_code = TEST_FAIL_S;

    return pRsp;
}

void* LGF_Testmode_ext_device_cmd(test_mode_req_type *pReq, DIAG_TEST_MODE_F_rsp_type *pRsp)
{
    if (diagpdev != NULL)
    {
        switch (pReq->ext_device_cmd)
        {
            case EXT_CARD_AUTO_TEST:
                testmode_result = 1;
                update_diagcmd_state(diagpdev, "EXT_CARD_AUTO_TEST", 0);
                msleep(500);

                if (testmode_result != -1)
                {
                    pRsp->ret_stat_code = TEST_OK_S;
                    pRsp->test_mode_rsp.uim_state = testmode_result;
                }
                else
                    pRsp->ret_stat_code = TEST_FAIL_S;
                break;

            default:
                pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
                break;
        }
    }
    else
        pRsp->ret_stat_code = TEST_FAIL_S;

    return pRsp;
}

static int test_mode_disable_input_devices = 0;
void LGF_TestModeSetDisableInputDevices(int value)
{
    test_mode_disable_input_devices = value;
}
int LGF_TestModeGetDisableInputDevices(void)
{
    return test_mode_disable_input_devices;
}
EXPORT_SYMBOL(LGF_TestModeGetDisableInputDevices);

void* LGF_TestModeKeyLockUnlock(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
    char buf[32];
    int len;
    pRsp->ret_stat_code = TEST_FAIL_S;

    switch(pReq->key_lock_unlock)
    {
        case KEY_LOCK:
            LGF_TestModeSetDisableInputDevices(1);

            len = sprintf(buf, "%d", 0);
            android_readwrite_file("/sys/class/leds/lcd-backlight/brightness", NULL, buf, len);

            pRsp->ret_stat_code = TEST_OK_S;
            break;

        case KEY_UNLOCK:
            LGF_TestModeSetDisableInputDevices(0);

            len = sprintf(buf, "%d", 100);
            android_readwrite_file("/sys/class/leds/lcd-backlight/brightness", NULL, buf, len);

            pRsp->ret_stat_code = TEST_OK_S;
            break;

        default:
            pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
            break;
    }

    return pRsp;
}

#ifndef SKW_TEST
static unsigned char test_mode_factory_reset_status = FACTORY_RESET_START;
#define BUF_PAGE_SIZE 2048
// BEGIN: 0010090 sehyuny.kim@lge.com 2010-10-21
// MOD 0010090: [FactoryReset] Enable Recovery mode FactoryReset

#define FACTORY_RESET_STR       "FACT_RESET_"
#define FACTORY_RESET_STR_SIZE	11
#define FACTORY_RESET_BLK 1 // read / write on the first block

#define MSLEEP_CNT 100

typedef struct MmcPartition MmcPartition;

struct MmcPartition {
    char *device_index;
    char *filesystem;
    char *name;
    unsigned dstatus;
    unsigned dtype ;
    unsigned dfirstsec;
    unsigned dsize;
};
// END: 0010090 sehyuny.kim@lge.com 2010-10-21
#endif

extern const MmcPartition *lge_mmc_find_partition_by_name(const char *name);

void* LGF_TestModeFactoryReset(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
 
  unsigned char pbuf[50]; //no need to have huge size, this is only for the flag
  const MmcPartition *pMisc_part; 
  unsigned char startStatus = FACTORY_RESET_NA; 
  int mtd_op_result = 0;
  unsigned long factoryreset_bytes_pos_in_emmc = 0;
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
  DIAG_TEST_MODE_F_req_type req_ptr;

  req_ptr.sub_cmd_code = TEST_MODE_FACTORY_RESET_CHECK_TEST;
  req_ptr.test_mode_req.factory_reset = pReq->factory_reset;
/* END: 0014656 jihoon.lee@lge.com 2011024 */
  
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
  pRsp->ret_stat_code = TEST_FAIL_S;
/* END: 0014110 jihoon.lee@lge.com 20110115 */
  
  lge_mmc_scan_partitions();
  pMisc_part = lge_mmc_find_partition_by_name("misc");
  factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;
  
  printk("LGF_TestModeFactoryReset> mmc info sec : 0x%x, size : 0x%x type : 0x%x frst sec: 0x%lx\n", pMisc_part->dfirstsec, pMisc_part->dsize, pMisc_part->dtype, factoryreset_bytes_pos_in_emmc);

/* BEGIN: 0013861 jihoon.lee@lge.com 20110111 */
/* MOD 0013861: [FACTORY RESET] emmc_direct_access factory reset flag access */
/* add carriage return and change flag size for the platform access */
/* END: 0013861 jihoon.lee@lge.com 20110111 */
  switch(pReq->factory_reset)
  {
    case FACTORY_RESET_CHECK :
#if 1  // def CONFIG_LGE_MTD_DIRECT_ACCESS
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      memset((void*)pbuf, 0, sizeof(pbuf));
      mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);

      if( mtd_op_result != (FACTORY_RESET_STR_SIZE+2) )
      {
        printk(KERN_ERR "[Testmode]lge_read_block, read data  = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
      else
      {
        //printk(KERN_INFO "\n[Testmode]factory reset memcmp\n");
        if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
        {
          startStatus = pbuf[FACTORY_RESET_STR_SIZE] - '0';
          printk(KERN_INFO "[Testmode]factory reset backup status = %d \n", startStatus);
        }
        else
        {
          // if the flag storage is erased this will be called, start from the initial state
          printk(KERN_ERR "[Testmode] tag read failed :  %s \n", pbuf);
        }
      }  
/* END: 0014110 jihoon.lee@lge.com 20110115 */

      test_mode_factory_reset_status = FACTORY_RESET_INITIAL;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d\n",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);

      mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, FACTORY_RESET_STR_SIZE+2);	
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(mtd_op_result!= (FACTORY_RESET_STR_SIZE+2))
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
      else
      {
        mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
        if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          pRsp->ret_stat_code = TEST_FAIL_S;
          break;
        }
      }
/* END: 0014110 jihoon.lee@lge.com 20110115 */

/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
      //send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      send_to_arm9((void*)&req_ptr, (void*)pRsp);
/* END: 0014656 jihoon.lee@lge.com 2011024 */

/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(pRsp->ret_stat_code != TEST_OK_S)
      {
        printk(KERN_ERR "[Testmode]send_to_arm9 response : %d\n", pRsp->ret_stat_code);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
/* END: 0014110 jihoon.lee@lge.com 20110115 */

      /*LG_FW khlee 2010.03.04 -If we start at 5, we have to go to APP reset state(3) directly */
      if((startStatus == FACTORY_RESET_COLD_BOOT_END) || (startStatus == FACTORY_RESET_HOME_SCREEN_END))
        test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      else
        test_mode_factory_reset_status = FACTORY_RESET_ARM9_END;

      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d\n",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);

      mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, FACTORY_RESET_STR_SIZE+2);
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
      else
      {
         mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
         if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
         {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          pRsp->ret_stat_code = TEST_FAIL_S;
          break;
         }
      }
/* END: 0014110 jihoon.lee@lge.com 20110115 */

#else /**/
      //send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      send_to_arm9((void*)&req_ptr, (void*)pRsp);
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/

      if((startStatus == FACTORY_RESET_COLD_BOOT_END) || (startStatus == FACTORY_RESET_HOME_SCREEN_END))
      {
        if (diagpdev != NULL)
          update_diagcmd_state(diagpdev, "REBOOT", 0);
        else
        {
          printk(KERN_INFO "%s, factory reset reboot failed \n", __func__);
          pRsp->ret_stat_code = TEST_FAIL_S;
          break;
        }
      }

      printk(KERN_INFO "%s, factory reset check completed \n", __func__);
      pRsp->ret_stat_code = TEST_OK_S;
      break;

    case FACTORY_RESET_COMPLETE_CHECK:

	 send_to_arm9((void*)&req_ptr, (void*)pRsp);
      if(pRsp->ret_stat_code != TEST_OK_S)
      {
        printk(KERN_ERR "[Testmode]send_to_arm9 response : %d\n", pRsp->ret_stat_code);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }

       break;

    case FACTORY_RESET_STATUS_CHECK:
#if 1 // def CONFIG_LGE_MTD_DIRECT_ACCESS
      memset((void*)pbuf, 0, sizeof(pbuf));
      mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2 );
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      {
      	 printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
      	 pRsp->ret_stat_code = TEST_FAIL_S;
      	 break;
      }
      else
      {
      	 if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
      	 {
      	   test_mode_factory_reset_status = pbuf[FACTORY_RESET_STR_SIZE] - '0';
      	   printk(KERN_INFO "[Testmode]factory reset status = %d \n", test_mode_factory_reset_status);
      	   pRsp->ret_stat_code = test_mode_factory_reset_status;
      	 }
      	 else
      	 {
      	   printk(KERN_ERR "[Testmode]factory reset tag fail, set initial state\n");
      	   test_mode_factory_reset_status = FACTORY_RESET_START;
      	   pRsp->ret_stat_code = test_mode_factory_reset_status;
      	   break;
      	 }
      }  
/* END: 0014110 jihoon.lee@lge.com 20110115 */
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/

      break;

    case FACTORY_RESET_COLD_BOOT:
// remove requesting sync to CP as all sync will be guaranteed on their own.

#if 1 // def CONFIG_LGE_MTD_DIRECT_ACCESS
      test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);
      mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc,  FACTORY_RESET_STR_SIZE+2);
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(mtd_op_result!=( FACTORY_RESET_STR_SIZE+2))
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
      else
      {
        mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf,  FACTORY_RESET_STR_SIZE+2);
        if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          pRsp->ret_stat_code = TEST_FAIL_S;
        }
      }
/* END: 0014110 jihoon.lee@lge.com 20110115 */
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
      pRsp->ret_stat_code = TEST_OK_S;
      break;

    case FACTORY_RESET_ERASE_USERDATA:
#if 1 // def CONFIG_LGE_MTD_DIRECT_ACCESS
      test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode-erase userdata]factory reset status = %d\n", test_mode_factory_reset_status);
      mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc , FACTORY_RESET_STR_SIZE+2);
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
      else
      {
        mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
        if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          pRsp->ret_stat_code = TEST_FAIL_S;
          break;
        }
      }
/* END: 0014110 jihoon.lee@lge.com 20110115 */
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
    pRsp->ret_stat_code = TEST_OK_S;
    break;
	

//   added New diag command  beacause  they want to skip facory reset when it was a factory download,
//   [250-50-4]
	case FACTORY_RESET_FORCE_CHANGE_STATUS: 

      test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_END;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode-force_change]factory reset status = %d\n", test_mode_factory_reset_status);

      mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc , FACTORY_RESET_STR_SIZE+2);
      if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
        break;
      }
      else
      {
        mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, pbuf, FACTORY_RESET_STR_SIZE+2);
        if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+2))
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          pRsp->ret_stat_code = TEST_FAIL_S;
          break;
        }
      }
   	 pRsp->ret_stat_code = TEST_OK_S;
    	break;
		

     default:
        pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
        break;
    }
 
  return pRsp;

}

//20110822 jungyub.jee@lge.com	SRD Backup[Start]
#define EMMC_SECTOR_UNIT 512
#define SRD_NV_ITEM_WRITE_BYTE_SIZE (EMMC_SECTOR_UNIT/2)
#define SRD_1MBYTE_AREA  (EMMC_SECTOR_UNIT*2048)
#define SRD_FIRST_AREA_INPARTITION 1  //nv
#define SRD_SECOND_AREA_INPARTITION 2  //extra nv
#define SRD_THIRD_AREA_INPARTITION 3  //prl 


void* LGF_TestModeBackup(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{

	  const MmcPartition *pMisc_part; 
	  int mtd_op_result = 0;
	  unsigned long srd_bytes_pos_in_emmc = 0;
	  unsigned long srd_nv_last_index =0;
	  int index =0;
	  char nv_index[2];
	  DIAG_TEST_MODE_F_req_type req_ptr;

  	 req_ptr.sub_cmd_code = TEST_MODE_SRD_BACKUP;

	  lge_mmc_scan_partitions();
	  pMisc_part = lge_mmc_find_partition_by_name("misc");
	  srd_bytes_pos_in_emmc  	= (pMisc_part->dfirstsec*512);
	  srd_nv_last_index		= srd_bytes_pos_in_emmc;
	  mtd_op_result = lge_erase_block(srd_bytes_pos_in_emmc, (size_t)(SRD_1MBYTE_AREA*SRD_FIRST_AREA_INPARTITION));
	  if(mtd_op_result != (SRD_1MBYTE_AREA*SRD_FIRST_AREA_INPARTITION))
	 {
          printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
          }
	  
	// srd_bytes_pos_in_emmc+0x400000  for  MDM 
	  mtd_op_result = lge_erase_block(srd_bytes_pos_in_emmc+0x400000, (size_t)(SRD_1MBYTE_AREA));
	  if(mtd_op_result != (SRD_1MBYTE_AREA*SRD_FIRST_AREA_INPARTITION))
	 {
          printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
          }
	 printk(KERN_INFO "SRD Start\n");
	for(;index<1500;index++)
	{
			send_to_arm9((void*)&req_ptr, (void*)pRsp);
			if(((DIAG_TEST_MODE_F_rsp_type*)pRsp)->ret_stat_code != TEST_FAIL_S)
			{

				mtd_op_result=lge_write_block(srd_bytes_pos_in_emmc+0x400000+512,pRsp->test_mode_rsp.srd_backup, 132);
				  if(mtd_op_result != 132)
				 {
			         	 printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
			         }
				srd_bytes_pos_in_emmc += 132;

			}
			else
			{
					printk(KERN_INFO "SRD Break\n");
					printk(KERN_INFO "SRD last index : %d\n",index);
					if(index != 0)
					{
						nv_index[0] = index/256;
						nv_index[1] = index%256;
						mtd_op_result=lge_write_block(srd_nv_last_index+0x400000+512+(132*1500),nv_index, 2);//nv item num write
						  if(mtd_op_result != 2)
						 {
					         	 printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
					         }
						pRsp->ret_stat_code = TEST_OK_S;
					}
					break;
			}
	}
		  return pRsp;


}
//20110822 jungyub.jee@lge.com	SRD Backup[End]

//20110823 jungyub.jee@lge.com	SRD Restore[Start]
void* LGF_TestModeRestore(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
	  const MmcPartition *pMisc_part; 
	  int mtd_op_result = 0;
	  unsigned long srd_bytes_pos_in_emmc = 0;
	  int nv_last_index =0;
	  int index =0;
	  char nv_index[2];
	  char nv_value[133];
	  DIAG_TEST_MODE_F_req_type req_ptr;

  	 req_ptr.sub_cmd_code = TEST_MODE_SRD_RESTORE;

	  lge_mmc_scan_partitions();
	  pMisc_part = lge_mmc_find_partition_by_name("misc");
	  srd_bytes_pos_in_emmc  	= (pMisc_part->dfirstsec*512);

	  mtd_op_result=lge_read_block(srd_bytes_pos_in_emmc+0x400000+512+(132*1500),nv_index, 2);//nb item num read
	   if(mtd_op_result != 2)
		{
	         	 printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
		}

	  nv_last_index = nv_index[0]*256 + nv_index[1];
	  for(;index<nv_last_index;index++)
	  {
	  	mtd_op_result=lge_read_block(srd_bytes_pos_in_emmc+0x400000+512,nv_value, 132);
		srd_bytes_pos_in_emmc += 132;
	       if(mtd_op_result != 132)
		{
	         	 printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
		}
		memcpy(req_ptr.test_mode_req.srd_resotre,nv_value,132);
		send_to_arm9((void*)&req_ptr, (void*)pRsp);
		
	  }
	  if(index == nv_last_index)
	  	pRsp->ret_stat_code = TEST_OK_S;
	   return pRsp;
	  

}
//20110823 jungyub.jee@lge.com	SRD Restore[End]

void* LGF_TestScriptItemSet(test_mode_req_type * pReq, DIAG_TEST_MODE_F_rsp_type * pRsp)
{
#if 1
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
  DIAG_TEST_MODE_F_req_type req_ptr;
  int mtd_op_result = 0;
  const MmcPartition *pMisc_part; 
  unsigned long factoryreset_bytes_pos_in_emmc = 0; 
//jaeo.park@lge.com [[
  udbp_req_type udbReqType;
  memset(&udbReqType,0x0,sizeof(udbp_req_type));
//]]
  

  req_ptr.sub_cmd_code = TEST_MODE_TEST_SCRIPT_MODE;
  req_ptr.test_mode_req.test_mode_test_scr_mode = pReq->test_mode_test_scr_mode;
/* END: 0014656 jihoon.lee@lge.com 2011024 */

  lge_mmc_scan_partitions();
  pMisc_part = lge_mmc_find_partition_by_name("misc");
  factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;
//  printk("LGF_TestScriptItemSet> mmc info sec : 0x%x, size : 0x%x type : 0x%x frst sec: 0x%lx\n", pMisc_part->dfirstsec, pMisc_part->dsize, pMisc_part->dtype, factoryreset_bytes_pos_in_emmc);

  switch(pReq->test_mode_test_scr_mode)
  {
    case TEST_SCRIPT_ITEM_SET:
	mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, (FACTORY_RESET_STR_SIZE+1) );	
	if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+1))
      {
      	 printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
      	 pRsp->ret_stat_code = TEST_FAIL_S;
      	 break;
	 }
		 
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
      	 //send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);

/* 20110831,hoyeon.jang@lge.com : only supported on skt device {++ */
#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
        send_to_arm9((void*)&req_ptr, (void*)pRsp);
#else
        pRsp->ret_stat_code = TEST_OK_S;
#endif
/* 20110831,hoyeon.jang@lge.com : only supported on skt device }++ */

        printk(KERN_INFO "%s, result : %s\n", __func__, pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
/* END: 0014656 jihoon.lee@lge.com 2011024 */
      break;

//jaeo.park@lge.com for SRD cal backup
    case CAL_DATA_BACKUP:
		udbReqType.header.sub_cmd = SRD_INIT_OPERATION;
		LGE_Dload_SRD((udbp_req_type *)&udbReqType,sizeof(udbReqType));//SRD_INIT_OPERATION
		udbReqType.header.sub_cmd = USERDATA_BACKUP_REQUEST;
		LGE_Dload_SRD((udbp_req_type *)&udbReqType,sizeof(udbReqType));//USERDATA_BACKUP_REQUEST
//		printk(KERN_INFO "%s,backup_nv_counter %d\n", __func__,userDataBackUpInfo.info.srd_backup_nv_counter);
		udbReqType.header.sub_cmd = USERDATA_BACKUP_REQUEST_MDM;
		LGE_Dload_SRD((udbp_req_type *)&udbReqType,sizeof(udbReqType));//USERDATA_BACKUP_REQUEST_MDM
//		printk(KERN_INFO "%s,backup_nv_counter %d\n", __func__,userDataBackUpInfo.info.srd_backup_nv_counter);
		break;
    
    case CAL_DATA_RESTORE:
		send_to_arm9((void*)&req_ptr, (void*)pRsp);
		printk(KERN_INFO "%s, result : %s\n", __func__, pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
		break;
/*
  	case CAL_DATA_ERASE:
  	case CAL_DATA_INFO:
  		diagpkt_free(pRsp);
  		return 0;			
  		break;
  */			
    default:
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
      //send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);

/* 20110831,hoyeon.jang@lge.com : only supported on skt device {++ */
#ifdef CONFIG_MACH_LGE_I_BOARD_SKT
      send_to_arm9((void*)&req_ptr, (void*)pRsp);
      printk(KERN_INFO "%s, cmd : %d, result : %s\n", __func__, pReq->test_mode_test_scr_mode, \
	  										pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
      if(pReq->test_mode_test_scr_mode == TEST_SCRIPT_MODE_CHECK)
      {
        switch(pRsp->test_mode_rsp.test_mode_test_scr_mode)
        {
          case 0:
            printk(KERN_INFO "%s, mode : %s\n", __func__, "USER SCRIPT");
            break;
          case 1:
            printk(KERN_INFO "%s, mode : %s\n", __func__, "TEST SCRIPT");
            break;
          default:
            printk(KERN_INFO "%s, mode : %s, returned %d\n", __func__, "NO PRL", pRsp->test_mode_rsp.test_mode_test_scr_mode);
            break;
        }
      }
#else
      pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
#endif
/* 20110831,hoyeon.jang@lge.com : only supported on skt device }++ */

/* END: 0014656 jihoon.lee@lge.com 2011024 */
      break;

  }  
        

// END: 0009720 sehyuny.kim@lge.com 2010-10-06

#else
// BEGIN: 0009720 sehyuny.kim@lge.com 2010-10-06
// MOD 0009720: [Modem] It add RF X-Backup feature
  int mtd_op_result = 0;

  const MmcPartition *pMisc_part; 
  unsigned long factoryreset_bytes_pos_in_emmc = 0;
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
  DIAG_TEST_MODE_F_req_type req_ptr;

  req_ptr.sub_cmd_code = TEST_MODE_TEST_SCRIPT_MODE;
  req_ptr.test_mode_req.test_mode_test_scr_mode = pReq->test_mode_test_scr_mode;
/* END: 0014656 jihoon.lee@lge.com 2011024 */

  lge_mmc_scan_partitions();
  pMisc_part = lge_mmc_find_partition_by_name("misc");
  factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;

  printk("LGF_TestScriptItemSet> mmc info sec : 0x%x, size : 0x%x type : 0x%x frst sec: 0x%lx\n", pMisc_part->dfirstsec, pMisc_part->dsize, pMisc_part->dtype, factoryreset_bytes_pos_in_emmc);

  switch(pReq->test_mode_test_scr_mode)
  {
    case TEST_SCRIPT_ITEM_SET:
  #if 1 // def CONFIG_LGE_MTD_DIRECT_ACCESS
      mtd_op_result = lge_erase_block(factoryreset_bytes_pos_in_emmc, (FACTORY_RESET_STR_SIZE+1) );
/* BEGIN: 0014110 jihoon.lee@lge.com 20110115 */
/* MOD 0014110: [FACTORY RESET] stability */
/* handle operation or rpc failure as well */
      if(mtd_op_result!=(FACTORY_RESET_STR_SIZE+1))
      {
      	 printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
      	 pRsp->ret_stat_code = TEST_FAIL_S;
      	 break;
/* END: 0014110 jihoon.lee@lge.com 20110115 */
      } else
  #endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
      // LG_FW khlee 2010.03.16 - They want to ACL on state in test script state.
      {
      	 update_diagcmd_state(diagpdev, "ALC", 1);
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
      	 //send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      	 send_to_arm9((void*)&req_ptr, (void*)pRsp);
        printk(KERN_INFO "%s, result : %s\n", __func__, pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
/* END: 0014656 jihoon.lee@lge.com 2011024 */
      }
      break;
  /*			
  	case CAL_DATA_BACKUP:
  	case CAL_DATA_RESTORE:
  	case CAL_DATA_ERASE:
  	case CAL_DATA_INFO:
  		diagpkt_free(pRsp);
  		return 0;			
  		break;
  */			
    default:
/* BEGIN: 0014656 jihoon.lee@lge.com 20110124 */
/* MOD 0014656: [LG RAPI] OEM RAPI PACKET MISMATCH KERNEL CRASH FIX */
      //send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      send_to_arm9((void*)&req_ptr, (void*)pRsp);
      printk(KERN_INFO "%s, cmd : %d, result : %s\n", __func__, pReq->test_mode_test_scr_mode, \
	  										pRsp->ret_stat_code==TEST_OK_S?"OK":"FALSE");
      if(pReq->test_mode_test_scr_mode == TEST_SCRIPT_MODE_CHECK)
      {
        switch(pRsp->test_mode_rsp.test_mode_test_scr_mode)
        {
          case 0:
            printk(KERN_INFO "%s, mode : %s\n", __func__, "USER SCRIPT");
            break;
          case 1:
            printk(KERN_INFO "%s, mode : %s\n", __func__, "TEST SCRIPT");
            break;
          default:
            printk(KERN_INFO "%s, mode : %s, returned %d\n", __func__, "NO PRL", pRsp->test_mode_rsp.test_mode_test_scr_mode);
            break;
        }
      }
/* END: 0014656 jihoon.lee@lge.com 2011024 */
      break;

  }  
// END: 0009720 sehyuny.kim@lge.com 2010-10-06
#endif 
  return pRsp;

}

//====================================================================
// Self Recovery Download Support  diag command 249-XX
//====================================================================
#ifdef CONFIG_LGE_DLOAD_SRD  //kabjoo.choi
PACK (void *)LGE_Dload_SRD (PACK (void *)req_pkt_ptr, uint16 pkg_len)
{

  	udbp_req_type		*req_ptr = (udbp_req_type *) req_pkt_ptr;
	udbp_rsp_type	  	*rsp_ptr = NULL;
	uint16 rsp_len = pkg_len;
	int write_size=0 , mtd_op_result=0;
	rsp_ptr = (udbp_rsp_type *)diagpkt_alloc(DIAG_USET_DATA_BACKUP, rsp_len);

  	// DIAG_TEST_MODE_F_rsp_type union type is greater than the actual size, decrease it in case sensitive items
  		switch(req_ptr->header.sub_cmd)
      		{
  			case  SRD_INIT_OPERATION:				
				diag_SRD_Init(req_ptr,rsp_ptr);							
				break;
				
			case USERDATA_BACKUP_REQUEST:
				printk(KERN_WARNING "USERDATA_BACKUP_REQUEST");
				//CSFB SRD remote 제거 -> diag_userDataBackUp_data 에서 처리함
				//remote_rpc_srd_cmmand(req_ptr, rsp_ptr);  //userDataBackUpStart() 여기서 ... shared ram 저장 하도록. .. 

				//CSFB SRD
				load_srd_base=kmalloc(SIZE_OF_SHARD_RAM, GFP_KERNEL);  //384K byte 
				diag_userDataBackUp_data(req_ptr, rsp_ptr);

				diag_userDataBackUp_entrySet(req_ptr,rsp_ptr,0);  //write info data ,  after rpc respons include write_sector_counter  

				//todo ..  rsp_prt->header.write_sector_counter,  how about checking  no active nv item  ; 
				// write ram data to emmc misc partition  as many as retruned setor counters 

				if (load_srd_base ==NULL)
				{
					printk(KERN_ERR "UDBU_ERROR_CANNOT_COMPLETE : load_srd_baseis NULL");
					((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_CANNOT_COMPLETE;	
					break;
					// return rsp_ptr;
				}
				  
				write_size= rsp_ptr->rsp_data.write_sector_counter *256;	 //return nv backup counters  

				if( write_size >SIZE_OF_SHARD_RAM)
				{
					printk(KERN_ERR "UDBU_ERROR_CANNOT_COMPLETE : write_size %d >SIZE_OF_SHARD_RAM %d", write_size, SIZE_OF_SHARD_RAM);
					((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_CANNOT_COMPLETE;  //hue..
					break;
				}

				//srd_bytes_pos_in_emmc+512 means that info data already writed at emmc first sector 
				printk(KERN_WARNING "[SRD]srd_bytes_pos_in_emmc = %d \n", srd_bytes_pos_in_emmc);
				mtd_op_result = lge_write_block(srd_bytes_pos_in_emmc+512, load_srd_base, write_size);  //512 info data
				
    			if(mtd_op_result!= write_size)
    			{
					printk(KERN_ERR "UDBU_ERROR_CANNOT_COMPLETE : mtd_op_result!= write_size");
					((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_CANNOT_COMPLETE;	
					kfree(load_srd_base);	//CSFB SRD
					break;
					//return rsp_ptr;
        		}
				kfree(load_srd_base);	//CSFB SRD
	 
				break;

			case USERDATA_BACKUP_REQUEST_MDM:
				//MDM backup 
//CSFB SRD
/*				((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_SUCCESS;	
				load_srd_shard_base=smem_alloc(SMEM_ERR_CRASH_LOG, SIZE_OF_SHARD_RAM);  //384K byte 
				
				if (load_srd_shard_base ==NULL)
				 {
				 	((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_CANNOT_COMPLETE;	
					break;
				 	// return rsp_ptr;
				 }	
				load_srd_shard_base+=1200*256 ; //mdm ram offset 
				
				// CSFB remote 제거 -> diag_userDataBackUp_data 에서 처리함
				//remote_rpc_srd_cmmand(req_ptr, rsp_ptr);  //userDataBackUpStart() 여기서 ... ram 저장 하도록. .. 
				diag_userDataBackUp_entrySet(req_ptr,rsp_ptr,1);  //write info data ,  after rpc respons include write_sector_counter  remote_rpc_srd_cmmand(req_ptr, rsp_ptr);  //userDataBackUpStart() 여기서 ... ram 저장 하도록. .. 
				write_size= rsp_ptr->rsp_data.write_sector_counter *256;	 //return nv backup counters  

				 if( write_size >0x15000)  //384K = mode ram (300K) + mdm (80K)
				 {
				 	((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_CANNOT_COMPLETE;  //hue..
				 	break;
				 }
				  load_srd_kernel_base=kmalloc((size_t)write_size, GFP_KERNEL);
				  memcpy(load_srd_kernel_base,load_srd_shard_base,write_size);	
				  
				 mtd_op_result = lge_write_block(srd_bytes_pos_in_emmc+0x400000+512, load_srd_kernel_base, write_size);  //not sector address > 4M byte offset  

				if(mtd_op_result!= write_size)
        			{
				((udbp_rsp_type*)rsp_ptr)->header.err_code = UDBU_ERROR_CANNOT_COMPLETE;	
				kfree(load_srd_kernel_base);
				break;
				//return rsp_ptr;
                   
        			}
				kfree(load_srd_kernel_base);*/
				break;

			case USERDATA_RESTORE_REQUEST :
				printk(KERN_WARNING "[SRD]USERDATA_RESTORE_REQUEST \n");
				rsp_ptr->header.cmd_code = req_ptr->header.cmd_code;
				rsp_ptr->header.sub_cmd = req_ptr->header.sub_cmd;
				rsp_ptr->header.packet_version = req_ptr->header.packet_version;
				rsp_ptr->header.dwsize = req_ptr->header.dwsize;
				rsp_ptr->header.err_code = 0; //UDBU_ERROR_SUCCESS
				if(srd_userDataRecovery() == FALSE)
					rsp_ptr->header.err_code = 1; //UDBU_ERROR_SUCCESS
				break;

			case GET_DOWNLOAD_INFO :
				break;

			case EXTRA_NV_OPERATION :
			#ifdef LG_FW_SRD_EXTRA_NV				
				diag_extraNv_entrySet(req_ptr,rsp_ptr);
			#endif
				break;
				
			case PRL_OPERATION :
			#ifdef LG_FW_SRD_PRL				
				diag_PRL_entrySet(req_ptr,rsp_ptr);
			#endif
				break;
				
			default :
  				rsp_ptr =NULL; //(void *) diagpkt_err_rsp (DIAG_BAD_PARM_F, req_ptr, pkt_len);
				break;
		
		}

	/* Execption*/	
	if (rsp_ptr == NULL){
		return NULL;
	}

  return rsp_ptr;
}
EXPORT_SYMBOL(LGE_Dload_SRD);
#endif 

/*  USAGE
 *  1. If you want to handle at ARM9 side, you have to insert fun_ptr as NULL and mark ARM9_PROCESSOR
 *  2. If you want to handle at ARM11 side , you have to insert fun_ptr as you want and mark AMR11_PROCESSOR.
 */

testmode_user_table_entry_type testmode_mstr_tbl[TESTMODE_MSTR_TBL_SIZE] =
{
    /* sub_command                          fun_ptr                           which procesor*/
    /* 0 ~ 10 */
    {TEST_MODE_VERSION,                     NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_LCD,                         not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_MOTOR,                       not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_ACOUSTIC,                    not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_CAM,                         not_supported_command_handler,    ARM11_PROCESSOR},
    /* 11 ~ 20 */
    {TEST_MODE_IRDA_FMRT_FINGER_UIM_TEST,   LGF_Testmode_ext_device_cmd,      ARM11_PROCESSOR},
    /* 21 ~ 30 */
    {TEST_MODE_KEY_TEST,                    not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_EXT_SOCKET_TEST,             LGF_ExternalSocketMemory,         ARM11_PROCESSOR},
    {TEST_MODE_BLUETOOTH_TEST,              not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_BATT_LEVEL_TEST,             LGF_TestModeBattLevel,            ARM11_PROCESSOR},
    {TEST_MODE_MP3_TEST,                    not_supported_command_handler,    ARM11_PROCESSOR},
    /* 31 ~ 40 */
    {TEST_MODE_ACCEL_SENSOR_TEST,           not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_WIFI_TEST,                   not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_MANUAL_TEST_MODE,            NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_FORMAT_MEMORY_TEST,          not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_KEY_DATA_TEST,               LGF_TestModeKeyData,              ARM11_PROCESSOR},
    /* 41 ~ 50 */
    {TEST_MODE_MEMORY_CAPA_TEST,            not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_SLEEP_MODE_TEST,             LGF_TestModeSleepMode,            ARM11_PROCESSOR},
    {TEST_MODE_SPEAKER_PHONE_TEST,          not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_VIRTUAL_SIM_TEST,            LGF_TestModeVirtualSimTest,       ARM11_PROCESSOR},
    {TEST_MODE_PHOTO_SENSER_TEST,           not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_MRD_USB_TEST,                NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_PROXIMITY_SENSOR_TEST,       not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_TEST_SCRIPT_MODE,            LGF_TestScriptItemSet,            ARM11_PROCESSOR},
    {TEST_MODE_FACTORY_RESET_CHECK_TEST,    LGF_TestModeFactoryReset,         ARM11_PROCESSOR},
    /* 51 ~60 */
    {TEST_MODE_VOLUME_TEST,                 not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_FIRST_BOOT_COMPLETE_TEST,    LGF_TestModeFBoot,                ARM11_PROCESSOR},
    {TEST_MODE_MAX_CURRENT_CHECK,           NULL,                             ARM9_PROCESSOR},
    /* 61 ~70 */
    {TEST_MODE_CHANGE_RFCALMODE,            NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_SELECT_MIMO_ANT,             NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_LTE_MODE_SELECTION,          not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_LTE_CALL,                    not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_CHANGE_USB_DRIVER,           not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_GET_HKADC_VALUE,             NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_LED_TEST,                    not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_PID_TEST,                    NULL,                             ARM9_PROCESSOR},
    /* 71 ~ 80 */
    {TEST_MODE_SW_VERSION,                  NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_IME_TEST,                    NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_IMPL_TEST,                   NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_SIM_LOCK_TYPE_TEST,          NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_UNLOCK_CODE_TEST,            NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_IDDE_TEST,                   NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_FULL_SIGNATURE_TEST,         NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_NT_CODE_TEST,                NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_SIM_ID_TEST,                 NULL,                             ARM9_PROCESSOR},
    /* 81 ~ 90*/
    {TEST_MODE_CAL_CHECK,                   NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_BLUETOOTH_RW,                NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_SKIP_WELCOM_TEST,            not_supported_command_handler,    ARM11_PROCESSOR},
    {TEST_MODE_WIFI_MAC_RW,                 LGF_TestModeWiFiMACRW,            ARM11_PROCESSOR},
    /* 91 ~ */
    {TEST_MODE_DB_INTEGRITY_CHECK,          LGF_TestModeDBIntegrityCheck,     ARM11_PROCESSOR},
    {TEST_MODE_NVCRC_CHECK,                 NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_RESET_PRODUCTION,            NULL,                             ARM9_PROCESSOR},
    {TEST_MODE_FOTA,                        LGF_TestModeFOTA,                 ARM11_PROCESSOR},
    {TEST_MODE_POWER_RESET,                 LGF_TestModePowerReset,           ARM11_PROCESSOR},
    {TEST_MODE_KEY_LOCK_UNLOCK,             LGF_TestModeKeyLockUnlock,        ARM11_PROCESSOR},
    {TEST_MODE_SRD_RESTORE,                 LGF_TestModeRestore,              ARM11_PROCESSOR},
    {TEST_MODE_SRD_BACKUP,                  LGF_TestModeBackup,               ARM11_PROCESSOR},
    {TEST_MODE_XO_CAL_DATA_COPY,            NULL,                             ARM9_PROCESSOR},
};
