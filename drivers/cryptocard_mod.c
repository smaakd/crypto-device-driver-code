
#include <linux/fs.h>
#include <linux/module.h>                         // MOD_DEVICE_TABLE,
#include <linux/init.h>
#include <linux/pci.h>                            // pci_device_id,
#include <linux/interrupt.h>  
#include <linux/uaccess.h>
#include <asm/io.h>     
#include <linux/io.h>                   // copy_to_user,
#include <linux/version.h>                        // KERNEL_VERSION,
#include <iso646.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/dmaengine.h>
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Saketh");
MODULE_DESCRIPTION("PCI device driver code");

#define VENDOR_ID 0x1234                          // vendor id: buggy Kolter card: official 0x1001 but real 0x0001
#define DEVICE_ID 0xdeba                          // device id of the Proto-3 card

// for first and second i/o region (pci memory):
static unsigned long ioport=0L, iolen=0L, memstart=0L, memlen=0L;
static int i_template_major = 231;
static void * device_access;
static char *kbuf;
// static dma_addr_t handle;

static struct semaphore *sem=NULL;
static struct semaphore *sem_lock=NULL;

static dma_addr_t dma_handle;
static void * cpu_addr ;

#define DEVNAME "crypto_device"

static int major;
atomic_t  device_opened;
static struct class *demo_class;
struct device *demo_device;
static struct pci_driver* devp;
static int config;
static unsigned int keya,keyb;

#define SET_KEY 0
#define SET_CONFIG 1
#define ENCRYPT 2
#define DECRYPT 3

#define TRUE 1
#define FALSE 0

#define SET 1
#define UNSET 0

#define LARGE_SIZE 32768 // 32KB 	

#define MMIO_NO_INT 0
#define MMIO_INT 1
#define DMA_NO_INT 2
#define DMA_INT 3
#define USER_SPACE 4

typedef enum {INTERRUPT, DMA} config_t;

struct write_to_op{
	int action;
	int cdev;
	uint8_t a;
	uint8_t b;
	config_t type;
	uint8_t value;
	void* addr;
	uint64_t length;
	uint8_t isMapped;
};


static struct
pci_device_id pci_drv_ids[] =
{
  // { VENDOR_ID, DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
  { PCI_DEVICE(VENDOR_ID, DEVICE_ID), },
  { 0, }
};
MODULE_DEVICE_TABLE(pci, pci_drv_ids);

// // declarations for fops, pci_driver
// static int _template_open (struct inode *, struct file *);
// static int _template_close (struct inode *, struct file *);
// static ssize_t _template_read (struct file *, char *, size_t, loff_t *);
// static ssize_t _template_write (struct file *, const char *, size_t, loff_t *);
static int device_init(struct pci_dev *, const struct pci_device_id *);
static void device_deinit( struct pci_dev *);
static void device_deinit( struct pci_dev *);
static int __init pci_drv_init(void);
static void __exit pci_drv_exit(void);

module_init(pci_drv_init);
module_exit(pci_drv_exit);

static struct
pci_driver pci_drv_template =
{
  .name= "pci_drv_template",
  .id_table= pci_drv_ids,
  .probe= device_init,
  .remove= device_deinit,
};


static void* encrypt_without_interrupt(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	if(buf == 0 ){
		char buf11[20];
		memcpy_fromio(buf11,(device_access + 0xa8),message_length);
		buf11[message_length] = '\0';
		pr_crit("before encrypted buf  = %s\n",(char *) buf11);
	}
	if(buf != 0){
		memcpy_toio((device_access + 0xa8), buf, message_length);
		char* buf1 = (char*) buf;
	}
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	iowrite32(message_length, (device_access + 0x0c));
	iowrite32(0x00 , (device_access + 0x20));
	iowrite32(0xa8 , (device_access + 0x80));
	while((ioread32(device_access + 0x20) & 0x1) != 0);
	if(buf != 0){
		memcpy_fromio(buf,(device_access + 0xa8),message_length);
	}
	if(buf == 0 ){
		char buf11[20];
		memcpy_fromio(buf11,(device_access + 0xa8),message_length);
		buf11[message_length] = '\0';
		pr_crit("after encrypted buf  = %s\n",(char *) buf11);
	}
	up(sem_lock);
	return buf;
}

static void* encrypt_without_interrupt_dma(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	if(buf != 0){
		strncpy(cpu_addr,buf,message_length);
		char* buff = cpu_addr;
	}
	writeq(message_length, (device_access + 0x98));
	writeq(dma_handle, (device_access + 0x90));
	writeq	(0x1, (device_access + 0xa0));
	while((readq(device_access + 0xa0) & 0x1) != 0){
	}
	if(buf != 0){
		strncpy(buf,cpu_addr,message_length);
	}
	up(sem_lock);
	return buf;
}

static void* encrypt_with_interrupt_dma(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	if(buf != 0){
		strncpy(cpu_addr,buf,message_length);
		char* buff = cpu_addr;
	}
	iowrite32(message_length, (device_access + 0x98));
	iowrite32(dma_handle, (device_access + 0x90));
	iowrite32(0x1|0x4, (device_access + 0xa0));
	down(sem);
	if(buf != 0){
		strncpy(buf,cpu_addr,message_length);
	}
	up(sem_lock);
	return buf;
}

static void* decrypt_without_interrupt_dma(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	writeq(keya|keyb, (device_access + 0x08));
	if(buf != 0){
		strncpy(cpu_addr,buf,message_length);
		char* buff = cpu_addr;
	}
	writeq(message_length, (device_access + 0x98));
	writeq(dma_handle, (device_access + 0x90));
	writeq(0x1|0x2, (device_access + 0xa0));
	while((readq(device_access + 0xa0) & 0x1) != 0);
	if(buf != 0){
		strncpy(buf,cpu_addr,message_length);
	}
	up(sem_lock);
	return buf;
}

static void* decrypt_with_interrupt_dma(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	if(buf != 0){
		strncpy(cpu_addr,buf,message_length);
		char* buff = cpu_addr;
	}
	iowrite32(message_length, (device_access + 0x98));
	iowrite32(dma_handle, (device_access + 0x90));
	iowrite32(0x1|0x2|0x4, (device_access + 0xa0));
	down(sem);
	if(buf != 0){
		strncpy(buf,cpu_addr,message_length);
	}
	up(sem_lock);
	return buf;
}

static void* encrypt_with_interrupt(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	if(buf != 0){
		memcpy_toio((device_access + 0xa8), buf, message_length);
		char* buf1 = (char*) buf;
	}
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	iowrite32(message_length, (device_access + 0x0c)); 
	if((ioread32(device_access + 0x20) & 0x1) == 0){
		iowrite32(0x00 | 0x80 , (device_access + 0x20));
		iowrite32(0xa8 , (device_access + 0x80));
	}
	down(sem);
	if(buf != 0){
		memcpy_fromio(buf,(device_access + 0xa8),message_length);
	}
	up(sem_lock);
	return buf;
}

static void* decrypt_without_interrupt(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	if(buf != 0){
		memcpy_toio((device_access + 0xa8), buf, message_length);
		char* buf1 = (char*) buf;
	}
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	iowrite32(message_length, (device_access + 0x0c));
	if((ioread32(device_access + 0x20) & 0x1) != 0);
	iowrite32(0x02 , (device_access + 0x20));
	iowrite32(0xa8 , (device_access + 0x80));
	while((ioread32(device_access + 0x20) & 0x1) != 0);
	if(buf != 0){
		memcpy_fromio(buf,(device_access + 0xa8),message_length);
	}
	up(sem_lock);
	return buf;
}

static void* decrypt_with_interrupt(unsigned int keya , unsigned int keyb ,void* buf , int message_length){
	down(sem_lock);
	if(buf != 0){
		memcpy_toio((device_access + 0xa8), buf, message_length);
		char* buf1 = (char*) buf;
	}
	keya = (keya << 8) & 0xff00;
	keyb = keyb & 0xff;
	iowrite32(keya|keyb, (device_access + 0x08));
	iowrite32(message_length, (device_access + 0x0c));
	if((ioread32(device_access + 0x20) & 0x1) != 0);
	iowrite32(0x02 | 0x80 , (device_access + 0x20));
	iowrite32(0xa8 , (device_access + 0x80));
	down(sem);
	if(buf != 0){
		memcpy_fromio(buf,(device_access + 0xa8),message_length);
	}
	up(sem_lock);
	return buf;
}


static
irqreturn_t pci_isr( int irq, void *dev_id)
{
	unsigned int res = ioread32(device_access + 0x24);
	iowrite32(res,(device_access + 0x64) );
	up(sem);
    return (IRQ_HANDLED);
}

static char *demo_devnode(struct device *dev, umode_t *mode)
{
        if (mode && dev->devt == MKDEV(i_template_major, 0))
                *mode = 0666;
        return NULL;
}


// Initialising of the module with output about the irq, I/O region and memory region.
static
int device_init(struct pci_dev *dev, const struct pci_device_id *id)
{
	config = MMIO_NO_INT;
  	int i_result,res;
  	u32 identification,val;
	res =  pci_enable_device( dev );
	sem = (struct semaphore *)kzalloc(sizeof(struct semaphore), GFP_KERNEL);
	sem_lock = (struct semaphore *)kzalloc(sizeof(struct semaphore), GFP_KERNEL);
	if(sem == NULL)
		pr_crit("sem uninitialized\n");
	if(sem_lock == NULL)
		pr_crit("sem uninitialized\n");	
	sema_init(sem, 0);
	sema_init(sem_lock, 1);
	cpu_addr = dma_alloc_coherent(&dev->dev, LARGE_SIZE, &dma_handle, GFP_DMA);
	if(!cpu_addr){
		pr_crit("cpu_addr is NULL");
	}
	if (devm_request_irq(&dev->dev, dev->irq, pci_isr, IRQF_SHARED, pci_drv_template.name, (void *) dev)){
        pr_crit("irq handler register failed!\n");
		goto cleanup_ports;
	}


	// get the first mmio region
	ioport = pci_resource_start( dev, 0 );
	iolen = pci_resource_len( dev, 0 );


	if(iolen){
		// char buf[20];
		// strncpy(buf,"hello world",11);
		// buf[11] = '\0';
		device_access = ioremap(ioport,iolen);
	}
	else{
		goto cleanup_irq;
	} 


	return(0) ; 
	cleanup_ports:
	if (iolen)
		release_region( ioport, iolen );
	cleanup_irq:
	if (dev->irq)
		free_irq( dev->irq, dev );
	return (-EIO);
}


static void device_deinit( struct pci_dev *pdev )
{
	dma_free_coherent(&pdev->dev, LARGE_SIZE, cpu_addr, dma_handle);
	iounmap( (void*)device_access ) ;
												  // device driver part
	unregister_chrdev (i_template_major, "pci_chrdev_template");
	if (pdev->irq)
		free_irq( pdev->irq, pdev );
	if( iolen )
		release_region( ioport, iolen );
}


static int demo_open(struct inode *inode, struct file *file)
{
        atomic_inc(&device_opened);
        try_module_get(THIS_MODULE);
        return 0;
}

static int demo_release(struct inode *inode, struct file *file)
{
        atomic_dec(&device_opened);
        module_put(THIS_MODULE);
        return 0;
}

static ssize_t _template_read (struct file * filp, char *buff, size_t count, loff_t * ppos)
{
	struct write_to_op* handle = (struct write_to_op*) buff;
	if(handle->action == SET_KEY){
		pr_err("keya = %d and keyb = %d",handle->a,handle->b);
		keya = (unsigned int) ((char) handle->a);
		keyb = (unsigned int) ((char) handle->b);
	}
	if(handle->action == SET_CONFIG){
		// config = handle->type;
		if((handle->type == SET) && (handle->value == SET)){
			if(config == MMIO_NO_INT){
					config = DMA_NO_INT;
				}
				else if(config == MMIO_INT){
					config = DMA_INT;
				}
				else if(config == DMA_NO_INT){
					config = DMA_NO_INT;
				}
				else if(config == DMA_INT){
					config = DMA_INT;
				}
		}
		else if((handle->type == SET) && (handle->value == UNSET)){
				if(config == MMIO_NO_INT){
					config = MMIO_NO_INT;
				}
				else if(config == MMIO_INT){
					config = MMIO_INT;
				}
				else if(config == DMA_NO_INT){
					config = MMIO_NO_INT;
				}
				else if(config == DMA_INT){
					config = MMIO_INT;
				}
		}
		else if((handle->type == UNSET) && (handle->value == SET)){
				if(config == MMIO_NO_INT){
					config = MMIO_INT;
				}
				else if(config == MMIO_INT){
					config = MMIO_INT;
				}
				else if(config == DMA_NO_INT){
					config = DMA_INT;
				}
				else if(config == DMA_INT){
					config = DMA_INT;
				}
		}
		else if((handle->type == UNSET) && (handle->value == UNSET)){
				if(config == MMIO_NO_INT){
					config = MMIO_NO_INT;
				}
				else if(config == MMIO_INT){
					config = MMIO_NO_INT;
				}
				else if(config == DMA_NO_INT){
					config = DMA_NO_INT;
				}
				else if(config == DMA_INT){
					config = DMA_NO_INT;
				}
		}
		else{
			return -1;
		}
	}
	if(handle->action == ENCRYPT){
		if(handle->isMapped == TRUE){
			int config_dup = config;
			u8 keya1 = handle->a;
			u8 keyb1 = handle->b;
			unsigned long len = (unsigned long)handle->length;
			if(config_dup == MMIO_NO_INT){
					encrypt_without_interrupt(keya1,keyb1,0,len);
				}
				else if(config_dup == MMIO_INT){
					encrypt_with_interrupt(keya1,keyb1,0,len);
				}
				else if(config_dup == DMA_NO_INT){
					encrypt_without_interrupt_dma(keya1,keyb1,0,len);
				}
				else if(config_dup == DMA_INT){
					encrypt_with_interrupt_dma(keya1,keyb1,0,len);
				}
		}
		else{
		int config_dup = config;
			u8 keya1 = handle->a;
			u8 keyb1 = handle->b;
			unsigned long len = (unsigned long)handle->length;
			 char * buf = (char*) kmalloc((len + 2) * sizeof(char), GFP_KERNEL);
			buf[(handle->length)] = '\0';
			copy_from_user(buf,handle->addr,len);
			int length;
			unsigned long curr;
			for (curr = 0; curr < len ; curr = curr + LARGE_SIZE){
				length = LARGE_SIZE;
				if((len - curr) < LARGE_SIZE){
					length = (int ) (len - curr) ;
				}
				if(config_dup == MMIO_NO_INT){
					encrypt_without_interrupt(keya1,keyb1,buf+curr,(int)length);
				}
				else if(config_dup == MMIO_INT){
					encrypt_with_interrupt(keya1,keyb1,buf+curr,(int)length);
				}
				else if(config_dup == DMA_NO_INT){
					encrypt_without_interrupt_dma(keya1,keyb1,buf+curr,(int)length);
				}
				else if(config_dup == DMA_INT){
					encrypt_with_interrupt_dma(keya1,keyb1,buf+curr,(int)length);
				}
			}
			stac();
			int res = copy_to_user(handle->addr,buf,len);
			clac();
		}
	}
	if(handle->action == DECRYPT){
		if(handle->isMapped == TRUE){
			int config_dup = config;
			u8 keya1 = handle->a;
			u8 keyb1 = handle->b;
			unsigned long len = (unsigned long)handle->length;
			if(config_dup == MMIO_NO_INT){
					decrypt_without_interrupt(keya1,keyb1,0,len);
				}
				else if(config_dup == MMIO_INT){
					decrypt_with_interrupt(keya1,keyb1,0,len);
				}
				else if(config_dup == DMA_NO_INT){
					decrypt_without_interrupt_dma(keya1,keyb1,0,len);
				}
				else if(config_dup == DMA_INT){
					decrypt_with_interrupt_dma(keya1,keyb1,0,len);
				}
		}
		else{
			int config_dup = config;
			uint8_t keya1 = handle->a;
			uint8_t keyb1 = handle->b;
			unsigned long len = (unsigned long)handle->length;
			char * buf = (char*) kmalloc((len + 2) * sizeof(char), GFP_KERNEL);
			buf[(handle->length)] = '\0';
			copy_from_user(buf,handle->addr,len);
			int length;
			unsigned long curr;
			for (curr = 0; curr < len ; curr = curr + LARGE_SIZE){
				length = LARGE_SIZE;
				if((len - curr) < LARGE_SIZE){
					length = (int ) (len - curr) ;
				}
				if(config_dup == MMIO_NO_INT){
					decrypt_without_interrupt(keya1,keyb1,buf+curr,(int)length);
				}
				else if(config_dup == MMIO_INT){
					decrypt_with_interrupt(keya1,keyb1,buf+curr,(int)length);
				}
				else if(config_dup == DMA_NO_INT){
					decrypt_without_interrupt_dma(keya1,keyb1,buf+curr,(int)length);
				}
				else if(config_dup == DMA_INT){
					decrypt_with_interrupt_dma(keya1,keyb1,buf+curr,(int)length);
				}
			}
			stac();
			int res = copy_to_user(handle->addr,buf,len);
			clac();
		}
	}
  return (9);
}



static ssize_t _template_write (struct file * filp,  const char *buff, size_t count, loff_t * ppos)
{
    return 8;
}

// struct page *simple_vma_nopage(struct vm_area_struct *vma,
//                 unsigned long address, int write_access)
// {
//     struct page *pageptr;
//     unsigned long physaddr = address - vma->vm_start + VMA_OFFSET(vma);
//     pageptr = virt_to_page(__va(physaddr));
//     get_page(pageptr);
//     return pageptr;
// }

static vm_fault_t handle_pte_fault(struct vm_fault *vmf){
	// pr_crit("fault_handler: hello\n");
	return -1;
}

static void simple_vma_open(struct vm_area_struct *vma)
{
    // printk(KERN_CRIT "Simple VMA open, virt %lx, phys %lx\n",
    //         vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void simple_vma_close(struct vm_area_struct *vma)
{
    // printk(KERN_ALERT "Simple VMA close.\n");
}

static struct vm_operations_struct simple_nopage_vm_ops = {
	.open =   simple_vma_open,
	.close =  simple_vma_close,
	.fault =  handle_pte_fault,
};

static pte_t* get_pte(unsigned long address)
{
        pgd_t *pgd;
        p4d_t *p4d;
        pud_t *pud;
        pmd_t *pmd;
        pte_t *ptep;
        struct mm_struct *mm = current->mm;
        struct vm_area_struct *vma = find_vma(mm, address);
        // if(!vma){
        //          printk(KERN_INFO "No vma yet\n");
        //          goto nul_ret;
        // }
       
        // *addr_vma = (unsigned long) vma;

        pgd = pgd_offset(mm, address);
        if (pgd_none(*pgd) || unlikely(pgd_bad(*pgd)))
                goto nul_ret;
        printk(KERN_INFO "pgd(va) [%lx] pgd (pa) [%lx] *pgd [%lx]\n", (unsigned long)pgd, __pa(pgd), pgd->pgd); 
        p4d = p4d_offset(pgd, address);
        if (p4d_none(*p4d))
                goto nul_ret;
        if (unlikely(p4d_bad(*p4d)))
                goto nul_ret;
        pud = pud_offset(p4d, address);
        if (pud_none(*pud))
                goto nul_ret;
        if (unlikely(pud_bad(*pud)))
                goto nul_ret;
        printk(KERN_INFO "pud(va) [%lx] pud (pa) [%lx] *pud [%lx]\n", (unsigned long)pud, __pa(pud), pud->pud); 

        pmd = pmd_offset(pud, address);
        if (pmd_none(*pmd))
                goto nul_ret;
        if (unlikely(pmd_trans_huge(*pmd))){
                printk(KERN_INFO "I am huge\n");
                goto nul_ret;
        }
        printk(KERN_INFO "pmd(va) [%lx] pmd (pa) [%lx] *pmd [%lx]\n", (unsigned long)pmd, __pa(pmd), pmd->pmd); 
        ptep = pte_offset_map(pmd, address);
        if(!ptep){
                printk(KERN_INFO "pte_p is null\n\n");
                goto nul_ret;
        }
        printk(KERN_INFO "pte(va) [%lx] pte (pa) [%lx] *pte [%lx]\n", (unsigned long)ptep, __pa(ptep), ptep->pte); 
        return ptep;

        nul_ret:
               printk(KERN_INFO "Address could not be translated\n");
               return NULL;

}


static int demo_mmap (struct file *filp, struct vm_area_struct * vma)
{
	// printk(KERN_ALERT "demo_mmap: reached!\n");
	// pr_err("size = %ld\n",(vma->vm_end-vma->vm_start));
	// pr_err("vm start = %lx\n",(vma->vm_start));
	// pr_err("addr1 = %lx\n",(device_access));
	// pr_err("addr2 = %lx\n",(device_access + 1));
	// get_pte((unsigned long) (device_access));
	// get_pte((unsigned long) (device_access + 1));
	// struct page *pgtable_t;
	// pr_err("prot = %x\n",vma->vm_page_prot);
	// pr_err("prot = %x\n",__pgprot(pgprot_noncached(vma->vm_page_prot).pgprot |  _PAGE_RW));
	// vma->vm_flags |= VM_IOREMAP;
	// vma->vm_flags |= VM_RESERVED;
	// vma->vm_page_prot = __pgprot(pgprot_device(vma->vm_page_prot).pgprot |  _PAGE_RW );
	vma->vm_page_prot = pgprot_device(vma->vm_page_prot);

	// pr_err("prot = %x\n",vma->vm_page_prot);
	if(remap_pfn_range(vma,vma->vm_start, (ioport) >> PAGE_SHIFT , 
            vma->vm_end-vma->vm_start, vma->vm_page_prot))
        return -EAGAIN;

	// if (ioremap_page_range(vma->vm_start, vma->vm_end, ioport, vma->vm_page_prot)){
	// 	pr_crit("ioremap_page_range: not working\n");
	// 	return -EAGAIN;
	// }

	// get_pte((unsigned long) (vma->vm_start));	
	vma->vm_ops = &simple_nopage_vm_ops;
	simple_vma_open(vma);
    return 0;	
}

static struct file_operations fops = {
        .read = _template_read,
        .write = _template_write,
        .open = demo_open,
        .release = demo_release,
		.mmap = demo_mmap,
};


// device driver init
static
int __init pci_drv_init(void)
{
	int err;
	// printk(KERN_INFO "Hello kernel\n");
            
        int major = register_chrdev(0, DEVNAME, &fops);
        err = major;
        if (err < 0) {      
             printk(KERN_ALERT "Registering char device failed with %d\n", major);   
             goto error_regdev;
        }                 
        i_template_major = major;
        demo_class = class_create(THIS_MODULE, DEVNAME);
        err = PTR_ERR(demo_class);
        if (IS_ERR(demo_class))
                goto error_class;

        demo_class->devnode = demo_devnode;

        demo_device = device_create(demo_class, NULL,
                                        MKDEV(major, 0),
                                        NULL, DEVNAME);
        err = PTR_ERR(demo_device);
        if (IS_ERR(demo_device))
                goto error_device;
 
        // printk(KERN_INFO "I was assigned major number %d. To talk to\n", major);                                                              
        atomic_set(&device_opened, 0);
	// return 0;
	config = 0;
	int res = pci_register_driver(&pci_drv_template);
	if(res == 0)
		return 0;
	device_destroy(demo_class, MKDEV(i_template_major, 0));
	error_device:
			class_destroy(demo_class);
	error_class:
			unregister_chrdev(major, DEVNAME);
	error_regdev:
			return  err;
}


static
void __exit pci_drv_exit(void)
{
	pci_unregister_driver(&pci_drv_template);
  device_destroy(demo_class, MKDEV(i_template_major, 0));
        class_destroy(demo_class);
        unregister_chrdev(i_template_major, DEVNAME);
	printk(KERN_INFO "Goodbye kernel\n");
}

