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


//读取指定的一个字节
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
uint8_t gd32_flash_read_halfword(uint32_t faddr)
{
    return *(volatile uint8_t*)faddr; 
}


//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
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
        writeaddr+=4;//地址增加2.
    }
}

/**
 * @brief 初始化文件系统的底层接口
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
// * @brief 申请临界互斥信号量
// * 
// * @param void
// * @return void
// */
//void fs_api_lock_init( void )
//{
//  fs_api_sem = xSemaphoreCreateMutex();
//}

///**
// * @brief 临界锁
// * 
// * @param void
// * @return void
// */
//void fs_api_lock( void )
//{
//    xSemaphoreTake( fs_api_sem, portMAX_DELAY );
//}

///**
// * @brief 临界解锁
// * 
// * @param void
// * @return void
// */
//void fs_api_unlock( void )
//{
//    xSemaphoreGive( fs_api_sem );
//}

/**
 * @brief 从flash读出数据
 * 
 * @param [in] c lfs_config数据结构
 * @param [in] block 要读的块
 * @param [in] off 在当前块的偏移
 * @param [out] buffer 读取到的数据
 * @param [in] size 要读取的字节数
 * @return 0 成功 <0 错误
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
 * @brief 数据写入flash
 * 
 * @param [in] c lfs_config数据结构
 * @param [in] block 要读的块
 * @param [in] off 在当前块的偏移
 * @param [out] buffer 读取到的数据
 * @param [in] size 要读取的字节数
 * @return 0 成功 <0 错误
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
 * @brief 擦除flash的一个block
 * 
 * @param [in] c lfs_config数据结构
 * @param [in] block 要擦除的块
 * @return 0 成功 <0 错误
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
 * @brief 同步存储接口
 * 
 * @param [in] c lfs_config数据结构
 * @return 0 成功 <0 错误
 */
static int32_t fs_api_flash_sync( const struct lfs_config *c )
{
    return 0;
}

/**
 * @brief 遍历文件并显示条目
 * 
 * @param [in] *lfs 文件系统句柄
 * @param [in] *path 文件路径
 * @return 0 成功 <0 错误
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
 * @brief 文件系统挂载
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
        //系统首次启动因为不存在该目录会打开失败，先格式化再尝试打开
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
 * @brief 显示目录
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
 * @brief 打开文件
 * 
 * @param [in] *fp 文件系统句柄
 * @param [in] *path 文件路径
 * @param [in] *mode 操作模式
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
 * @brief 关闭文件
 * 
 * @param [in] *fp 文件系统句柄
 * @return 0 成功 <0 错误
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
 * @brief 读文件
 * 
 * @param [in] *fp 文件系统句柄
 * @param [in] size 读出的数据大小。
 * @param [out] *dst 读出的数据缓存
 * @return 0 成功 <0 错误
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
 * @brief 写文件
 * 
 * @param [in] *fp 文件系统句柄
 * @param [in] size 写入的数据大小。
 * @param [in] *src 写入的数据缓存
 * @return 0 成功 <0 错误
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
 * @brief 修改文件光标位置
 * 
 * @param [in] *fp 文件系统句柄
 * @param [in] offset 偏移量
 * @param [in] from 从哪开始
 * @return 0 成功 <0 错误
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
 * @brief 获取文件大小
 * 
 * @param [in] *fp 文件系统句柄
 * @return 0 成功 <0 错误
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
 * @brief 删除文件或目录
 * 
 * @param [in] *path 文件路径
 * @return 0 成功 <0 错误
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
 * @brief 格式化文件系统
 * 
 * @param [in] *path 文件路径
 * @return 0 成功 <0 错误
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
 * @brief 截断文件
 * 
 * @param [in] sise 截断后的文件大小
 * @return 0 成功 <0 错误
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
