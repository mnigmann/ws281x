#include <linux/module.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/uaccess.h>    /* for put_user */
#include "rpi_dma_utils.h"
#include "ws281x.h"

MODULE_LICENSE("GPL");

static void my_release(struct device *dev) {
	pr_info("releasing DMA device\n");
}


static struct device dev = {
	.release = my_release
};

uint64_t dma_mask;

// Session variables
static int n_open = 0;
struct ws281x_config conf;
uint32_t dmamem_size;

dma_addr_t physAddr;
void *virtAddr;
DMA_CB *cbp;

uint32_t colors[] = {0xFF0000, 0xFF9900, 0xCCFF00, 0x33FF00, 0x00FF66, 0x00FFFF, 0x0066FF, 0x3300FF, 0xCC00FF, 0xFF0099};
#define N_LEDS 10


__iomem void *ioMemory;
__iomem void *dma_reg;
__iomem void *smi_reg;
__iomem void *clk_reg;

#define SETBITS(mask, addr) writel(readl(addr) | (mask), addr)
#define WRITEBITS(mask, value, addr) writel((readl(addr) & (~(mask))) | value, addr)
#define BUS_DMA_MEM(x) ((uint32_t)(x) - (uint32_t)virtAddr + (uint32_t)physAddr)


// Initialise SMI, given data width, time step, and setup/hold/strobe counts
// Step value is in nanoseconds: even numbers, 2 to 30
void init_smi(int width, int ns, int setup, int strobe, int hold) {
    int divi = ns / 2;

    
    //smi_cs->value = smi_l->value = smi_a->value = 0;
    writel(1<<4, smi_reg+SMI_CS);
    writel(0, smi_reg+SMI_L);
    writel(0, smi_reg+SMI_A);
    writel(0, smi_reg+SMI_DSR);
    writel(0, smi_reg+SMI_DSW);
    writel(0, smi_reg+SMI_DCS);
    writel(0, smi_reg+SMI_DCA);

    if (readl(clk_reg+CLK_SMI_DIV) != divi << 12) {
        writel(CLK_PASSWD | (1<<5), clk_reg+CLK_SMI_CTL);
        while (readl(clk_reg+CLK_SMI_CTL) & (1 << 7)) ;
        writel(CLK_PASSWD | (divi << 12), clk_reg+CLK_SMI_DIV);
        writel(CLK_PASSWD | 6 | (1<<4), clk_reg+CLK_SMI_CTL);
        while ((readl(clk_reg+CLK_SMI_CTL) & (1 << 7)) == 0) ;
    }
    uint32_t smi_cs = readl(smi_reg+SMI_CS);
    if (smi_cs & (1<<7)) writel(smi_cs | (1<<7), smi_reg+SMI_CS);
    
    writel(strobe | (hold << 16) | (setup << 24) | (width << 30), smi_reg+SMI_DSR);
    writel(strobe | (hold << 16) | (setup << 24) | (width << 30), smi_reg+SMI_DSW);
    writel((8 << 12) | (8 << 18) | (4 << 6) | 4, smi_reg+SMI_DMC);

    //smi_dsr->rsetup = smi_dsw->wsetup = setup;
    //smi_dsr->rstrobe = smi_dsw->wstrobe = strobe;
    //smi_dsr->rhold = smi_dsw->whold = hold;
    //smi_dmc->panicr = smi_dmc->panicw = 8;
    //smi_dmc->reqr = smi_dmc->reqw = 2;
    //smi_dsr->rwidth = smi_dsw->wwidth = width;
    
    SETBITS((1<<4) | (1<<13) | (1<<25) | (1<<14), smi_reg+SMI_CS);
    SETBITS(1, smi_reg+SMI_DCS);
    WRITEBITS(3<<30, SMI_8_BITS<<30, smi_reg+SMI_DSR);
    writel(72*conf.stringlen, smi_reg+SMI_L);
    //writel(16, smi_reg+SMI_L);
    SETBITS(1<<28, smi_reg+SMI_DMC);
    SETBITS(1<<5, smi_reg+SMI_CS);
    SETBITS(1<<4, smi_reg+SMI_CS);
}


void update_leds(void) {
    if (!virtAddr) return;

    DMA_CB *cbs = virtAddr;
    uint8_t *txdata = (uint8_t *)(cbs+1);

    

    SETBITS(1 << DMA_CHAN, dma_reg+DMA_ENABLE);
    writel(1<<31, dma_reg+DMA_CS);
    memset(cbs, 0, sizeof(DMA_CB));
    cbs[0].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC;
    cbs[0].tfr_len = 72*conf.stringlen;
    cbs[0].srce_ad = BUS_DMA_MEM(txdata);
    cbs[0].dest_ad = SMI_BASE-PHYS_REG_BASE+BUS_REG_BASE+SMI_D;
    cbs[0].next_cb = BUS_DMA_MEM(cbs+1);

    /*cbs[0].ti = DMA_CB_SRCE_INC;
    cbs[0].tfr_len = 4;
    cbs[0].srce_ad = BUS_DMA_MEM(txdata);
    cbs[0].dest_ad = GPIO_BASE-PHYS_REG_BASE+BUS_REG_BASE+GPIO_SET0;
    cbs[0].next_cb = BUS_DMA_MEM(cbs+1);
    cbs[1].tfr_len = 4;
    cbs[1].srce_ad = BUS_DMA_MEM(txdata);
    cbs[1].dest_ad = GPIO_BASE - PHYS_REG_BASE + BUS_REG_BASE + GPIO_CLR0;
    cbs[1].next_cb = BUS_DMA_MEM(cbs);*/

    writel(BUS_DMA_MEM(cbs), dma_reg+DMA_CONBLK_AD);
    writel(2, dma_reg+DMA_CS);       // Clear 'end' flag
    writel(7, dma_reg+DMA_DEBUG);    // Clear error bits
    writel(1, dma_reg+DMA_CS);       // Start DMA  */
    SETBITS(1<<4, smi_reg+SMI_CS);
    //writel(0x54004031, smi_reg+SMI_CS);
    writel(72*conf.stringlen, smi_reg+SMI_L);
    writel(0x54004021, smi_reg+SMI_CS);
    SETBITS(1<<3, smi_reg+SMI_CS);
    //writel(0x54004029, smi_reg+SMI_CS);
}

/*
 * Called when a process tries to open the device file, like
 * "cat /dev/mycharfile"
 */
static int device_open(struct inode *inode, struct file *file) {
        if (n_open) {
            printk(KERN_INFO "open failed");
            return -EBUSY;
        }
        n_open++;

        printk(KERN_INFO "opening\n");
        try_module_get(THIS_MODULE);

        return 0;
}

/*
 * Called when a process closes the device file.
 */
static int device_release(struct inode *inode, struct file *file)
{
        n_open--;          /* We're now ready for our next caller */

        /*
         * Decrement the usage count, or else once you opened the file, you'll
         * never get get rid of the module.
         */
        module_put(THIS_MODULE);

        return 0;
}


static ssize_t device_write(struct file *file, const char __user *buf, size_t length, loff_t *offset) {
    if (!virtAddr) return 1;
    //printk(KERN_INFO "write detected; SMI length %d; status %08X, length %d\n", readl(smi_reg+SMI_L), readl(smi_reg+SMI_CS), length);
    uint8_t *txdata = (uint8_t *)(virtAddr + sizeof(DMA_CB));
    uint32_t i, j;
    uint32_t k = 0;
    uint8_t shift;
    uint8_t mask = 0x50;
    uint32_t u = 0;
    uint32_t n_ch = 0;
    for (i=conf.mask; i > 0; i = i >> 1) n_ch += (i&1);

    // TODO: make this part compatible with correctly sized memory
    // currently, this part only fills the first string
    memset(txdata, 0, dmamem_size-sizeof(DMA_CB));
    for (i=0; i < length && i < 3*conf.stringlen*n_ch; i+=3) {
        copy_from_user(&k, buf+i, 3);
        mask = (i >= 3*conf.stringlen ? 0x40 : 0x10);
        u = i % (3*conf.stringlen);
        // Green pixels
        for (j=0; j < 8; j++) {
            txdata[(24*u+3*j)^1] |= mask;
            if ((k >> (15-j)) & 1) txdata[(24*u+3*j+1)^1] |= mask;
            //txdata[(24*i+3*j+2)^1] = 0;
        }
        // Red pixels
        for (j=0; j < 8; j++) {
            txdata[(24*u+3*j+24)^1] |= mask;
            if ((k >> (7-j)) & 1) txdata[(24*u+3*j+25)^1] |= mask;
            //txdata[(24*i+3*j+26)^1] = 0;
        }
        // Blue pixels
        for (j=0; j < 8; j++) {
            txdata[(24*u+3*j+48)^1] |= mask;
            if ((k >> (23-j)) & 1) txdata[(24*u+3*j+49)^1] |= mask;
            //txdata[(24*i+3*j+50)^1] = 0;
        }
    }

    init_smi(0, 10, 10, 20, 10);
    if (conf.flags & WS281X_AUTO_UPDATE) {
        uint32_t prev = readl(smi_reg+SMI_L);
        uint32_t prev_dma = readl(dma_reg+DMA_TXFR_LEN);
        uint32_t prev_cs = readl(smi_reg+SMI_CS);
/*        if (!(prev_dma & 0x00000002)) {
            printk(KERN_INFO "CS %08X,      CONBLK_AD %08X, TI %08X,        SOURCE_AD %08X\n", readl(dma_reg+DMA_CS), readl(dma_reg+DMA_CONBLK_AD), readl(dma_reg+DMA_TI), readl(dma_reg+DMA_SRCE_AD));
            printk(KERN_INFO "DEST_AD %08X, TXFR_LEN %08X,  NEXTCONBK %08X, DEBUG %08X\n", readl(dma_reg+DMA_DEST_AD), readl(dma_reg+DMA_TXFR_LEN), readl(dma_reg+DMA_NEXTCONBK), readl(dma_reg+DMA_DEBUG));
            //return -EBUSY;
        }*/
        update_leds();
        // printk(KERN_INFO "SMI length before: %d, SMI length after: %d, status %08X -> %08X, DMA %08X -> %08X\n", prev, readl(smi_reg+SMI_L), prev_cs, readl(smi_reg+SMI_CS), prev_dma, readl(dma_reg+DMA_TXFR_LEN));
        /*SETBITS(1<<3, smi_reg+SMI_CS);
        writel(*((uint32_t*)txdata), smi_reg+SMI_D);
        writel(*(1+(uint32_t*)txdata), smi_reg+SMI_D);
        writel(*(2+(uint32_t*)txdata), smi_reg+SMI_D);
        writel(*(3+(uint32_t*)txdata), smi_reg+SMI_D);*/
    }
    return 0;
}

static loff_t device_seek(struct file *file, loff_t offset, int whence) {
    return 0;
}

long int device_ioctl(struct file *file, unsigned int num, unsigned long arg) {
    switch (num) {
        case IOCTL_CONFIG:
            copy_from_user(&conf, (void *)arg, sizeof(conf));
            writel(72*conf.stringlen, smi_reg+SMI_L);
            conf.flags = conf.flags & 0x7FFFFFFF;
            if ((conf.mask & 0xFF) == conf.mask) conf.flags |= WS281X_USE_8BIT;
            printk(KERN_INFO "ioctl received flags: %08X, stringlen: %d, mask: %08X\n", conf.flags, conf.stringlen, conf.mask);
            uint32_t i = conf.mask, n=0;
            for (; i>0; i=i>>1) n+=(i&1);
            
            // Deallocate previous memory
            if (virtAddr) dma_free_coherent(&dev, dmamem_size, virtAddr, physAddr);
            dev.dma_mask = &dma_mask;
            if (dma_set_mask_and_coherent(&dev, DMA_BIT_MASK(32))) {
		printk(KERN_INFO "No suitable DMA available\n");
	    }            
            // Allocate DMA memory
            dmamem_size = PAGE_ROUNDUP(((conf.flags & WS281X_USE_8BIT) ? 24 : 48)*(conf.flags & WS281X_BYTES_PER_LED)*conf.stringlen + sizeof(DMA_CB));
            virtAddr = dma_alloc_coherent(&dev, dmamem_size, &physAddr, GFP_KERNEL);
            if (!(virtAddr)) {
                printk(KERN_INFO "Failed to allocate memory");
            } else {
                memset(virtAddr, 0, dmamem_size);
                update_leds();
                printk(KERN_INFO "virt 0x%08x, phys 0x%08x, allocated %d bytes, length %d\n", (uint32_t)virtAddr, physAddr, dmamem_size, readl(smi_reg+SMI_L));
            }
            break;
        case IOCTL_CHCONFIG:
            break;
        case IOCTL_UPDATE:
            update_leds();
            break;
    }
    return 0;
}


static struct file_operations fops = {
        .write = device_write,
        .open = device_open,
        .release = device_release,
        .llseek = device_seek,
        .unlocked_ioctl = device_ioctl
};


void dac_ladder_write(int val) {
    printk(KERN_INFO "writing %d", val);
    SETBITS(0xC, smi_reg+SMI_DCS);
    writel(val & 0xff, smi_reg+SMI_DCD);
    SETBITS(0x2, smi_reg+SMI_DCS);
}


int init_module() {
    printk(KERN_INFO "starting\n");

    dev_set_name(&dev, "dmatest");
    device_register(&dev);

    register_chrdev(MAJOR_NUM, DEVICE_FILE_NAME, &fops);

    printk(KERN_INFO "allocating memory, preferred ioctl is %08X\n", IOCTL_CONFIG);
    // Set IO settings
    ioMemory = ioremap(GPIO_BASE, PAGE_SIZE);	
    //writel((readl(ioMemory) & (~(7 << 24))) | (1 << 24), ioMemory);
    //writel(1<<8, ioMemory+GPIO_SET0);
    //writel(1<<8, ioMemory+GPIO_CLR0);
    //writel(1<<8, ioMemory+GPIO_SET0);
    //writel(1<<8, ioMemory+GPIO_CLR0);
    //writel((readl(ioMemory+0x04) & (~(7 << 12))) | (5 << 12), ioMemory+0x04);
    WRITEBITS((7<<12) | (7<<6), (5<<12) | (5<<6), ioMemory+0x04);
    WRITEBITS((7<<21) | (7<<18), (5<<21) | (5<<18), ioMemory);
    writel(1<<17, ioMemory + 0x1c);


    // Map memory for DMA, SMI, and CLK
    dma_reg = ioremap(DMA_BASE, PAGE_SIZE);
    smi_reg = ioremap(SMI_BASE, PAGE_SIZE);
    clk_reg = ioremap(CLK_BASE, PAGE_SIZE);
    init_smi(0, 10, 10, 20, 10);

    //update_leds();

    /*int i;
    for (i=0; i < 16; i+=1) {
        dac_ladder_write(i);
        udelay(10);
    }*/

    /*SETBITS(1<<3, smi_reg+SMI_CS);
    writel(0xFFFF, smi_reg+SMI_D);
    writel(0x0000, smi_reg+SMI_D);
    writel(0xFFFF, smi_reg+SMI_D);
    writel(0x0000, smi_reg+SMI_D);*/

    printk(KERN_INFO "DEBUG: %08x, CS: %08x, SMI_CS: %08x\n", readl(dma_reg+DMA_DEBUG), readl(dma_reg+DMA_CS), readl(smi_reg+SMI_CS));

    return 0;
}	

void cleanup_module() {
	if (virtAddr) dma_free_coherent(&dev, PAGE_SIZE, virtAddr, physAddr);     // Deallocate DMA memory
    SETBITS(1<<31, dma_reg+DMA_CS);     // Stop DMA
    writel(0, smi_reg+SMI_CS);          // Disable SMI
    iounmap(ioMemory);                  // Unmap memory
    iounmap(dma_reg);
    iounmap(smi_reg);
    iounmap(clk_reg);
    unregister_chrdev(MAJOR_NUM, DEVICE_FILE_NAME);
	device_unregister(&dev);
	printk(KERN_INFO "stopping\n");
}

