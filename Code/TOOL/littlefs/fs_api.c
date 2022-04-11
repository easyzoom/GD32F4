#include "fs_api.h"
//#include "cmsis_os.h"
//#include "semphr.h"

static int32_t fs_api_flash_write( const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                             lfs_size_t size );
static int32_t fs_api_flash_erase( const struct lfs_config *c, lfs_block_t block );
static int32_t fs_api_flash_sync( const struct lfs_config *c );
int32_t fs_api_flash_read( const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size );

//static SemaphoreHandle_t    fs_api_sem;
//static FILE_SYSTEM_STR          fs_api_fs;
FILE_SYSTEM_STR          fs_api_fs;

__align(4) static uint8_t read_buffer[16];
__align(4) static uint8_t prog_buffer[16];
__align(4) static uint8_t lookahead_buffer[16];

const struct lfs_config lfs_cfg =
{
    // block device operations
    .read  = fs_api_flash_read,
    .prog  = fs_api_flash_write,
    .erase = fs_api_flash_erase,
    .sync  = fs_api_flash_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = GD32_FLASH_ERASE_GRAN,
    .block_count = GD32_FLASH_NUM_GRAN,
    .cache_size = 16,
    .lookahead_size =  16,
    .block_cycles = 500,
    
    .read_buffer = read_buffer,
    .prog_buffer = prog_buffer,
    .lookahead_buffer = lookahead_buffer,
};


//��ȡָ����һ���ֽ�
//faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
uint8_t gd32_flash_read_halfword(uint32_t faddr)
{
    return *(volatile uint8_t*)faddr; 
}


//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void gd32_flash_write_nocheck(uint32_t writeaddr, uint8_t *pbuffer, uint32_t numtowrite)   
{
    for(uint32_t i=0; i < numtowrite; i++)
    {
        fmc_byte_program(writeaddr, pbuffer[i]);
        writeaddr++;
    }
}


void gd32_flash_erase(uint32_t writeaddr, uint16_t numtowrite)   
{
    uint16_t i;
    uint32_t addr;
    addr = (writeaddr - GD32_FLASH_FLLESYS_START_BASE) / GD32_FLASH_ERASE_GRAN;
    addr = GD32_FLASH_FLLESYS_START_BASE + addr * GD32_FLASH_ERASE_GRAN;
    for(i=0; i < numtowrite / 4; i++)
    {
        fmc_word_program(addr,0xffff);
        writeaddr+=4;//��ַ����2.
    }
}

/**
 * @brief ��ʼ���ļ�ϵͳ�ĵײ�ӿ�
 * 
 * @param void
 * @return void
 */
uint8_t fs_api_init( void )
{
    int8_t state = 0;
    
//    fs_api_lock_init();
    state = fs_api_mount();

    if(state >= 0)
    {
        state = fs_api_ls();
    }
    if(state >= 0)
    {
        state = 0;
        printf("File system init success\r\n");
    }
    else
    {
        state = 1;
        printf("File system init failure\r\n");
    }

    return state;
}

///**
// * @brief �����ٽ绥���ź���
// * 
// * @param void
// * @return void
// */
//void fs_api_lock_init( void )
//{
//  fs_api_sem = xSemaphoreCreateMutex();
//}

///**
// * @brief �ٽ���
// * 
// * @param void
// * @return void
// */
//void fs_api_lock( void )
//{
//    xSemaphoreTake( fs_api_sem, portMAX_DELAY );
//}

///**
// * @brief �ٽ����
// * 
// * @param void
// * @return void
// */
//void fs_api_unlock( void )
//{
//    xSemaphoreGive( fs_api_sem );
//}

/**
 * @brief ��flash��������
 * 
 * @param [in] c lfs_config���ݽṹ
 * @param [in] block Ҫ���Ŀ�
 * @param [in] off �ڵ�ǰ���ƫ��
 * @param [out] buffer ��ȡ��������
 * @param [in] size Ҫ��ȡ���ֽ���
 * @return 0 �ɹ� <0 ����
 */
int32_t fs_api_flash_read( const struct lfs_config *c, lfs_block_t block,  lfs_off_t off, void *buffer, lfs_size_t size )
{
    uint8_t *temp = buffer;
    uint32_t addr = 0;

    addr = GD32_FLASH_FLLESYS_START_BASE + c->block_size * block + off;
    
    for(int i =0; i < size; i++)
    {
        temp[i] = gd32_flash_read_halfword(addr);
        addr++;
    }
    
    return 0;
}

/**
 * @brief ����д��flash
 * 
 * @param [in] c lfs_config���ݽṹ
 * @param [in] block Ҫ���Ŀ�
 * @param [in] off �ڵ�ǰ���ƫ��
 * @param [out] buffer ��ȡ��������
 * @param [in] size Ҫ��ȡ���ֽ���
 * @return 0 �ɹ� <0 ����
 */
static int32_t fs_api_flash_write( const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                             lfs_size_t size )
{
    uint32_t addr = 0;

    addr = GD32_FLASH_FLLESYS_START_BASE + c->block_size * block + off;
    fmc_unlock();
    gd32_flash_write_nocheck(addr, (uint8_t *)buffer, size);
    fmc_lock();
    
    return 0;
}

/**
 * @brief ����flash��һ��block
 * 
 * @param [in] c lfs_config���ݽṹ
 * @param [in] block Ҫ�����Ŀ�
 * @return 0 �ɹ� <0 ����
 */
static int32_t fs_api_flash_erase( const struct lfs_config *c, lfs_block_t block )
{
    uint32_t sector = block + 8;

    printf("bk:%d\r\n", block);
    switch(sector)
    {
        case 8:
            fmc_sector_erase(CTL_SECTOR_NUMBER_8);
        break;
        case 9:
            fmc_sector_erase(CTL_SECTOR_NUMBER_9);
        break;
        case 10:
            fmc_sector_erase(CTL_SECTOR_NUMBER_10);
        break;
        case 11:
            fmc_sector_erase(CTL_SECTOR_NUMBER_11);
        break;
    }
    
//    uint32_t addr = GD32_FLASH_FLLESYS_START_BASE + c->block_size * block;
//    fmc_unlock();
//    gd32_flash_erase(addr, c->block_size);
//    fmc_lock();
    return 0;
}

/**
 * @brief ͬ���洢�ӿ�
 * 
 * @param [in] c lfs_config���ݽṹ
 * @return 0 �ɹ� <0 ����
 */
static int32_t fs_api_flash_sync( const struct lfs_config *c )
{
    return 0;
}

/**
 * @brief �����ļ�����ʾ��Ŀ
 * 
 * @param [in] *lfs �ļ�ϵͳ���
 * @param [in] *path �ļ�·��
 * @return 0 �ɹ� <0 ����
 */
int fs_api_lfs_ls( FILE_SYSTEM_STR *lfs, const char *path )
{
        struct lfs_info info;
        lfs_dir_t fs_api_dir;
    
    int err = lfs_dir_open( lfs, &fs_api_dir, path );
    if( err )
    {
        return err;
    }
        
    while( true )
    {
        int res = lfs_dir_read( lfs, &fs_api_dir, &info );
        if( res < 0 )
        {
            return res;
        }
        if( res == 0 )
        {
            break;
        }
        printf( "\t%s", info.name );
        switch( info.type )
        {
            case LFS_TYPE_REG:
                printf( "\t\t\t\t%u Byte \r\n", info.size );
                break;
            case LFS_TYPE_DIR:
                printf( "\t\t\t\t%s Dir\r\n" ,info.name);
                break;
            default:
                printf( "?\r\n" );
                break;
        }
    }
    err = lfs_dir_close( lfs, &fs_api_dir );
    if( err )
    {
        return err;
    }
        
    return 0;
}

/**
 * @brief �ļ�ϵͳ����
 * 
 * @param void
 * @return void
 */
int fs_api_mount( void )
{
    FILE_STR fp;
    
    int err;

    err = lfs_mount( &fs_api_fs, &lfs_cfg );
    if( !err )
    {
        err = fs_api_fopen( &fp, FILE_BASIC_INFO, "r" );
        if( !err )
        {
                err = fs_api_fclose(&fp);
        }
        
        err = fs_api_fopen( &fp, FILE_PORT_MANAGE, "r" );
        if( !err )
        {
                err = fs_api_fclose(&fp);
        }
    }
        //ϵͳ�״�������Ϊ�����ڸ�Ŀ¼���ʧ�ܣ��ȸ�ʽ���ٳ��Դ�
    if( err )
    {
        printf( "File system start Format!!\r\n" );
        err = lfs_format( &fs_api_fs, &lfs_cfg );
        if(err < 0)
        {
            printf( "File system format failure!!\r\n");
            return -1;
        }
        err = lfs_mount( &fs_api_fs, &lfs_cfg );
        if(err < 0)
        {
            printf( "File system mount failure!!\r\n");
            return -2;
        }
        err =fs_api_fopen( &fp, FILE_BASIC_INFO, "c" );
        if(err < 0)
        {
            printf( "FILE_BASIC_INFO creat failure!!\r\n");
            return -3;
        }
        err = fs_api_fclose(&fp);
        if(err < 0)
        {
            printf( "FILE_BASIC_INFO close failure!!\r\n");
            return -4;
        }
        err =fs_api_fopen( &fp, FILE_PORT_MANAGE, "c" );
        if(err < 0)
        {
            printf( "FILE_PORT_MANAGE creat failure!!\r\n");
            return -5;
        }
        err = fs_api_fclose(&fp);
        if(err < 0)
        {
            printf( "FILE_PORT_MANAGE close failure!!\r\n");
            return -6;
        }
    }
    printf( "File system mounted\r\n" );
    
    return 0;
}

/**
 * @brief ��ʾĿ¼
 * 
 * @param void
 * @return void
 */
#define ls    fs_api_ls
int fs_api_ls( void )
{
        int err = 0;
    
    printf( "\r\nfiles on [\\]\r\n" );
//    fs_api_lock();
    err = fs_api_lfs_ls( &fs_api_fs,  "/" );
//    fs_api_unlock();
    printf( "\r\n\r\n" );
    
        return err;
}

/**
 * @brief ���ļ�
 * 
 * @param [in] *fp �ļ�ϵͳ���
 * @param [in] *path �ļ�·��
 * @param [in] *mode ����ģʽ
 * @return void
 */
int fs_api_fopen( FILE_STR *fp, char *path, char *mode )
{
    int res = 0;
    uint32_t create = 0, modify = 0, state = 0;
    uint32_t wr = 0;
    
    if( strstr( mode, "c" ) != 0 )
        create = 1;
    if( strstr( mode, "+" ) != 0 )
        modify = 1;
    if( strstr( mode, "wr" ) != 0 )
        wr = 1;
    
    if( create )
        state |= (LFS_O_RDWR |LFS_O_CREAT);
    
    if( modify )
        state |= LFS_O_APPEND;
    
    if( wr )
        state |= LFS_O_RDWR;
    
    if( state == 0 )
        state |= LFS_O_RDONLY;

    
    
//    fs_api_lock();
    res = lfs_file_open( &fs_api_fs, fp, path, state );
//    fs_api_unlock();
    if( res )
        res = -1;

//        printf("\nFOPEN(%s, %s) = %d\r\n", path, mode, res);
        
    return res;
}

/**
 * @brief �ر��ļ�
 * 
 * @param [in] *fp �ļ�ϵͳ���
 * @return 0 �ɹ� <0 ����
 */
int fs_api_fclose( FILE_STR *fp )
{
    int res = 0;
//    printf("\nFCLOSE(fp:%d)\r\n", *fp);

//    fs_api_lock();
    res = lfs_file_close( &fs_api_fs, fp );
//    fs_api_unlock();

    return res;
}

/**
 * @brief ���ļ�
 * 
 * @param [in] *fp �ļ�ϵͳ���
 * @param [in] size ���������ݴ�С��
 * @param [out] *dst ���������ݻ���
 * @return 0 �ɹ� <0 ����
 */
int32_t fs_api_fread( FILE_STR *fp, uint32_t size, void *dst )
{
    int32_t ret = 0;

//    fs_api_lock();
    ret = lfs_file_read( &fs_api_fs, fp, dst, size );
//    fs_api_unlock();

    printf("\r\nFREAD(fp:%08x, %u) = %d b\r\n", fp, size, ret);
    return ret;
}

/**
 * @brief д�ļ�
 * 
 * @param [in] *fp �ļ�ϵͳ���
 * @param [in] size д������ݴ�С��
 * @param [in] *src д������ݻ���
 * @return 0 �ɹ� <0 ����
 */
int32_t fs_api_fwrite( FILE_STR *fp, uint32_t size, void *src )
{
    uint32_t ret = 0;

//    fs_api_lock();
    ret = lfs_file_write( &fs_api_fs, fp, src, size );
//    fs_api_unlock();

    printf("\nFWRITE(fp:%08x, %d) = %d\r\n", fp, size, ret);
    
    return ret;
}

/**
 * @brief �޸��ļ����λ��
 * 
 * @param [in] *fp �ļ�ϵͳ���
 * @param [in] offset ƫ����
 * @param [in] from ���Ŀ�ʼ
 * @return 0 �ɹ� <0 ����
 */
int32_t fs_api_fseek( FILE_STR *fp, uint32_t offset, uint32_t from )
{
    uint32_t p;

//    fs_api_lock();
    p = lfs_file_seek( &fs_api_fs, fp, offset, from );
//    fs_api_unlock();
    printf("\nFSEEK(fp:0x%08x, %d, %d) = %d\r\n", fp, offset, from, p);
    
    return p;
}

/**
 * @brief ��ȡ�ļ���С
 * 
 * @param [in] *fp �ļ�ϵͳ���
 * @return 0 �ɹ� <0 ����
 */
int32_t fs_api_fsize( FILE_STR *fp )
{
    uint32_t ret;
    
   // ret = fs_api_fseek( fp, 0, LFS_SEEK_END );
        ret = lfs_file_size(&fs_api_fs, fp);
        printf("\nFSIZE(fp:0x%08x) = %d\n", fp, ret);
    
    return ret;
}

/**
 * @brief ɾ���ļ���Ŀ¼
 * 
 * @param [in] *path �ļ�·��
 * @return 0 �ɹ� <0 ����
 */
int fs_api_fremove( char *path )
{
    int res = 0;

//    fs_api_lock();
    res = lfs_remove( &fs_api_fs, path );
//    fs_api_unlock();

    return res;
}

/**
 * @brief ��ʽ���ļ�ϵͳ
 * 
 * @param [in] *path �ļ�·��
 * @return 0 �ɹ� <0 ����
 */
int fs_api_format(void)
{
    int res = 0;

    res = lfs_format( &fs_api_fs, &lfs_cfg );

    if(res < 0)
    {
        res = -1;
    }
    
    return res;
}

/**
 * @brief �ض��ļ�
 * 
 * @param [in] sise �ضϺ���ļ���С
 * @return 0 �ɹ� <0 ����
 */
int fs_api_truncate(FILE_STR *fp ,uint32_t size)
{
    int res = 0;

    res = lfs_file_truncate( &fs_api_fs, fp ,size);

    if(res < 0)
    {
        res = -1;
    }
    
    return res;
}


void lfs_test(void)
{
    FILE_STR fp;

    int err = lfs_mount(&fs_api_fs, &lfs_cfg);
    if (err)
    {
        lfs_format(&fs_api_fs, &lfs_cfg);
        lfs_mount(&fs_api_fs, &lfs_cfg);
    }
    uint32_t boot_count = 0;
    lfs_file_open(&fs_api_fs, &fp, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&fs_api_fs, &fp, &boot_count, sizeof(boot_count));
    boot_count += 1;
    lfs_file_rewind(&fs_api_fs, &fp);
    lfs_file_write(&fs_api_fs, &fp, &boot_count, sizeof(boot_count));
    lfs_file_close(&fs_api_fs, &fp);
//    lfs_unmount(&fs_api_fs);
    printf("boot_count: %d\r\n", boot_count);
}
