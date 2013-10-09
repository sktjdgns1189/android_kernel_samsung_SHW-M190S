/* ehci-s5pv210.c - Driver for USB HOST on Samsung S5PV210 processor
 *
 * Bus Glue for SAMSUNG S5PV210 USB HOST EHCI Controller
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * Based on "ehci-au1xxx.c" by by K.Boge <karsten.boge@amd.com>
 * Modified for SAMSUNG s5pv210 EHCI by Jingoo Han <jg1.han@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/clk.h>
#include <linux/platform_device.h>

static struct clk *usb_clk;
static struct usb_hcd  *g_hcd; /* for test */
static struct platform_device *g_pdev;
extern int usb_disabled(void);

//extern int get_boot_charger_info(void);
extern bool charging_mode_get(void);

extern void usb_host_phy_init(void);
extern void usb_host_phy_off(void);
//extern int ldo38_control_by_usbhost_pmstate(bool onoff);
	
static void s5pv210_start_ehc(void);
static void s5pv210_stop_ehc(void);
static int ehci_hcd_s5pv210_drv_probe(struct platform_device *pdev);
static int ehci_hcd_s5pv210_drv_remove(struct platform_device *pdev);

#ifdef CONFIG_PM
static int ehci_hcd_s5pv210_drv_suspend(
	struct platform_device *pdev,
	pm_message_t message
){
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	unsigned long flags;
	int rc = 0;

	printk("%s.\n", __func__);

#if 1 /* ehci removed case */
	if(hcd==NULL)
	{
		printk("hcd is null\n");
		return 0;
	}
	else if(hcd->state==HC_STATE_HALT)
	{
		printk("hcd state is HC_STATE_HALT\n");
		return 0;
	}
#endif

	//ldo38_control_by_usbhost_pmstate(1);

	//printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(hcd->regs + 0x90));
	//printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(hcd->regs + 0x94));

	if (time_before(jiffies, ehci->next_statechange)) {
		printk("[ehci_hcd_s5pv210_drv_suspend] jiffies time out!, jiffies = %ld, ehci->next_statechange = %ld\n", jiffies, ehci->next_statechange);
		msleep(10);
	}

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */

	//printk("[ehci_hcd_s5pv210_drv_suspend] hcd->state = 0x%x\n", hcd->state);

	// HC_STATE_SUSPENDED is __SUSPEND (0x4)
	spin_lock_irqsave(&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		printk("hcd->state is not HC_STATE_SUSPENDED, goto bail\n");
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void) ehci_readl(ehci, &ehci->regs->intr_enable);
	//printk("[ehci_hcd_s5pv210_drv_suspend] read intr_enable = %d\n", ehci_readl(ehci, &ehci->regs->intr_enable));

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW) {
		//printk("ehci_halt\n");
		(void) ehci_halt(ehci);
		//printk("ehci_reset\n");
		(void) ehci_reset(ehci);
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	//printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(hcd->regs + 0x90));
	//printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(hcd->regs + 0x94));

	s5pv210_stop_ehc();
	//mdelay(10); //marginal time before ldo38 off.
bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	//printk("ehci_hcd_s5pv210_drv_suspend(), ldo38 off\n");
	//ldo38_control_by_usbhost_pmstate(0); /* pair with ehci_bus_suspend */
	//printk("%s <<\n", __func__);

	return rc;
}
static void ehci_hcd_regs_init(void)
{
	//printk("PKD@%s\n",__func__);
//#ifdef CONFIG_CPU_S5PV210_EVT1
#if 1
	writel(0x000E0000, g_hcd->regs + 0x90);
	writel(0x00400040, g_hcd->regs + 0x94);
	// specifies the programmable microframe base value.
	//printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(g_hcd->regs + 0x90));
	// specifies the programmable packet buffer OUT/IN thresholds.
	//printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(g_hcd->regs + 0x94));
#else
	writel(0x00600040, g_hcd->regs + 0x94);
#endif
}
static int ehci_hcd_s5pv210_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	printk("%s.\n", __func__);

#if 1 /* ehci removed case */
	if(hcd==NULL)
	{
		printk("hcd is null\n");
		return 0;
	}
	else if(hcd->state==HC_STATE_HALT)
	{
		printk("hcd state is HC_STATE_HALT\n");
		return 0;
	}
#endif

	//ldo38_control_by_usbhost_pmstate(1); /* pair with ehci_bus_resume */
	s5pv210_start_ehc();
	//msleep(10); //marginal time before accessing registers.
	//PKD@251110
	ehci_hcd_regs_init();
	if (time_before(jiffies, ehci->next_statechange)) { //jiffies time out ???
		printk("[ehci_hcd_s5pv210_drv_resume] jiffies time out!, jiffies = %ld, ehci->next_statechange = %ld\n", jiffies, ehci->next_statechange);
		msleep(10);
	}

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	//printk("[ehci_hcd_s5pv210_drv_resume] read configured_flag = %d\n", ehci_readl(ehci, &ehci->regs->configured_flag));
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		(void) ehci_readl(ehci, &ehci->regs->intr_enable);
		printk("[ehci_hcd_s5pv210_drv_resume] read intr_enable = %d, return 0\n", ehci->regs->intr_enable);
		//ldo38_control_by_usbhost_pmstate(0);
		return 0;
	}

	//ehci_dbg(ehci, "lost power, restarting\n");
	usb_root_hub_lost_power(hcd->self.root_hub);

	//printk("ehci_halt..\n");
	(void) ehci_halt(ehci);
	//printk("ehci_reset..\n");
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	//printk("write 0x%x to reg 0x%x\n",ehci->command, &ehci->regs->command);
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	(void) ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */
	//printk("read reg 0x%x, value is 0x%x\n", &ehci->regs->command, ehci_readl(ehci, &ehci->regs->command));

	/* here we "know" root ports should always stay powered */
	//printk("ehci_port_power..\n");
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;

	//printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(hcd->regs + 0x90));
	//printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(hcd->regs + 0x94));
	//ldo38_control_by_usbhost_pmstate(0);
	//printk("%s <<\n", __func__);
	
	return 0;
}

#else
#define ehci_hcd_s5pv210_drv_suspend NULL
#define ehci_hcd_s5pv210_drv_resume NULL
#endif

static void s5pv210_start_ehc(void)
{
	//printk("s5pv210_start_ehc()......\n");
	clk_enable(usb_clk);
	usb_host_phy_init();
}

static void s5pv210_stop_ehc(void)
{
	//printk("s5pv210_stop_ehc()......\n");
	usb_host_phy_off();
	clk_disable(usb_clk);
}

#if 0
int check_usb_host_operation(void)
{
	int i = 0;
	int ret = 0;
	int prev_frame=0, cur_frame=0;

	for(i=0; i<5; i++)
	{
		msleep(10);
		cur_frame = g_hcd->driver->get_frame_number(g_hcd);
		printk("[FRINDEX-%d] %d\n", i, cur_frame);
		if(i==0) {
			prev_frame = cur_frame;
		}
		else {
			if(prev_frame!=cur_frame) {
				ret = 1; //host OK
				break;
			}
			else {
				prev_frame = cur_frame;
			}
		}
	}

	/* if ret is 0, it's not OK */
	return ret;
}
EXPORT_SYMBOL(check_usb_host_operation);
#endif

#if 0 /* for test */
void ehci_control_for_vusboff(void)
{
	//PKD@ EHCI spec says if s/w clears 0th bit of USBCMD
	// host controller should halt within 16 micro-frames.
	int i = 0;
	// USBCMD : clear 0-bit
	writel(readl(g_hcd->regs + 0x10)&~(0x1<<0), g_hcd->regs + 0x10);
	// USBSTS : check 12-bit 
	while(1) {
		usleep(125);
		if(readl(g_hcd->regs + 0x14)&(0x1<<12)) {
			printk("USBSTS 12-bit is 1\n");
			break;
		}
		else if(i > 16) {
			printk("check USBSTS time out!\n");
			break;
		}
		i++;
	}
}
EXPORT_SYMBOL(ehci_control_for_vusboff);

void ehci_control_for_vusbon(void)
{
	int i = 0;
	//PKD@ EHCI spec says we can write 1 to USBCMD[0] (Run/Stop bit)
	// till USBSTS[12] (HCHalted) is 1.
	// USBCMD : set 0-bit only if HCHalted of USBSTS is 1
	if(readl(g_hcd->regs + 0x14) & (0x1<<12))
		writel(readl(g_hcd->regs + 0x10)|(0x1<<0), g_hcd->regs + 0x10);
	else
		printk("Error!!! %s\n",__func__);

	//PKD@ Does following check is really required???
	// USBSTS : check 12-bit 
	while(1) {
		msleep(100);
		if(!(readl(g_hcd->regs + 0x14)&(0x1<<12))) {
			printk("USBSTS 12-bit is 0\n");
			break;
		}
		else if(i > 10) {
			printk("check USBSTS time out!\n");
			break;
		}
		i++;
	}
}
EXPORT_SYMBOL(ehci_control_for_vusbon);

int ehci_force_suspend(void)
{
	struct ehci_hcd *ehci = hcd_to_ehci(g_hcd);
	unsigned long flags;
	int rc = 0;

	//ldo38_control_by_usbhost_pmstate(1);

	printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(g_hcd->regs + 0x90));
	printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(g_hcd->regs + 0x94));

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */

	//printk("[ehci_hcd_s5pv210_drv_suspend] hcd->state = 0x%x\n", hcd->state);

	//g_hcd->state = HC_STATE_SUSPENDED;
	
	// HC_STATE_SUSPENDED is __SUSPEND (0x4)
	spin_lock_irqsave(&ehci->lock, flags);
	//if (g_hcd->state != HC_STATE_SUSPENDED) {
	//	printk("hcd->state is not HC_STATE_SUSPENDED, goto bail\n");
	//	rc = -EINVAL;
	//	goto bail;
	//}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void) ehci_readl(ehci, &ehci->regs->intr_enable);
	//printk("[ehci_hcd_s5pv210_drv_suspend] read intr_enable = %d\n", ehci_readl(ehci, &ehci->regs->intr_enable));

	/* make sure snapshot being resumed re-enumerates everything */
	//printk("ehci_halt\n");
	//(void) ehci_halt(ehci);
	//printk("ehci_reset\n");
	//(void) ehci_reset(ehci);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &g_hcd->flags);

	//printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(hcd->regs + 0x90));
	//printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(hcd->regs + 0x94));

	s5pv210_stop_ehc();
	mdelay(10); //marginal time before ldo38 off.
bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	printk("ehci_hcd_s5pv210_drv_suspend_mode()\n");
	//ldo38_control_by_usbhost_pmstate(0);

	return rc;
}
EXPORT_SYMBOL(ehci_force_suspend);

int ehci_force_resume(void)
{
	struct ehci_hcd *ehci = hcd_to_ehci(g_hcd);

	//ldo38_control_by_usbhost_pmstate(1);
	s5pv210_start_ehc();
	msleep(10); //marginal time before accessing registers.

	// fix that client deosn't work after suspend and wakeup.
	writel(0x000E0000, g_hcd->regs + 0x90);
	writel(0x00400040, g_hcd->regs + 0x94);

	// specifies the programmable microframe base value.
	printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(g_hcd->regs + 0x90));
	// specifies the programmable packet buffer OUT/IN thresholds.
	printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(g_hcd->regs + 0x94));

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &g_hcd->flags);

	//printk("[ehci_hcd_s5pv210_drv_resume] read configured_flag = %d\n", ehci_readl(ehci, &ehci->regs->configured_flag));
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!g_hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		(void) ehci_readl(ehci, &ehci->regs->intr_enable);
		printk("[ehci_hcd_s5pv210_drv_resume] read intr_enable = %d, return 0\n", ehci->regs->intr_enable);
		//ldo38_control_by_usbhost_pmstate(0);
		return 0;
	}

	//ehci_dbg(ehci, "lost power, restarting\n");
	usb_root_hub_lost_power(g_hcd->self.root_hub);

	//printk("ehci_halt..\n");
	(void) ehci_halt(ehci);
	//printk("ehci_reset..\n");
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	//printk("write 0x%x to reg 0x%x\n",ehci->command, &ehci->regs->command);
	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	(void) ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */
	//printk("read reg 0x%x, value is 0x%x\n", &ehci->regs->command, ehci_readl(ehci, &ehci->regs->command));

	/* here we "know" root ports should always stay powered */
	//printk("ehci_port_power..\n");
	ehci_port_power(ehci, 1);

	g_hcd->state = HC_STATE_SUSPENDED;

	//printk("read hcd->regs + 0x90 = 0x%x (0x000E0000)\n", readl(hcd->regs + 0x90));
	//printk("read hcd->regs + 0x94 = 0x%x (0x00400040)\n", readl(hcd->regs + 0x94));
	//ldo38_control_by_usbhost_pmstate(0);
	
	return 0;
}
EXPORT_SYMBOL(ehci_force_resume);
#endif /* for test */

static const struct hc_driver ehci_s5pv210_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "s5pv210 EHCI",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	.irq			= ehci_irq,
	.flags			= HCD_MEMORY|HCD_USB2,

	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	.get_frame_number	= ehci_get_frame,

	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

//extern struct delayed_work *ehci_vusb_work;
//extern void ehci_check_vusb_off(struct work_struct *work);

static int ehci_hcd_s5pv210_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = NULL;
	struct ehci_hcd *ehci = NULL;
	int retval = 0;
	g_pdev = pdev;

	printk("[ehci] ehci_hcd_s5pv210_drv_probe...\n");
	
	//if (get_boot_charger_info())
	if (charging_mode_get()) {
		printk("[ehci] is lpm-boot, %d\n", charging_mode_get());
		return -ENODEV;
	}
		
	if (usb_disabled()) {
		printk("[ehci] error! no usb\n");
		return -ENODEV;
	}

	//printk("[echi] ehci_hcd_s5pv210_drv_probe, usb_host_phy_off()\n");
	usb_host_phy_off();

	if (pdev->resource[1].flags != IORESOURCE_IRQ) {
		dev_err(&pdev->dev, "resource[1] is not IORESOURCE_IRQ.\n");
		return -ENODEV;
	}

	hcd = usb_create_hcd(&ehci_s5pv210_hc_driver, &pdev->dev, "s5pv210");
	if (!hcd) {
		dev_err(&pdev->dev, "usb_create_hcd failed!\n");
		return -ENODEV;
	}

	g_hcd = hcd; /* for test */

	printk("hcd->rsrc_start & end = 0x%x, 0x%x\n", pdev->resource[0].start, pdev->resource[0].end);
	
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_err(&pdev->dev, "request_mem_region failed!\n");
		retval = -EBUSY;
		goto err1;
	}

	usb_clk = clk_get(&pdev->dev, "usb-host");
	if (IS_ERR(usb_clk)) {
		dev_err(&pdev->dev, "cannot get usb-host clock\n");
		retval = -ENODEV;
		goto err2;
	}

	//printk("[echi] ehci_hcd_s5pv210_drv_probe, s5pv210_start_ehc()\n");
	s5pv210_start_ehc();

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "ioremap failed!\n");
		retval = -ENOMEM;
		goto err2;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	ehci_hcd_regs_init();
	retval = usb_add_hcd(hcd, pdev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);

	if (retval == 0) {
		platform_set_drvdata(pdev, hcd);
		//schedule_delayed_work(ehci_vusb_work, msecs_to_jiffies(1000));
		return retval;
	}

	printk("[echi] ehci_hcd_s5pv210_drv_probe, s5pv210_stop_ehc()\n");
	s5pv210_stop_ehc();
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	clk_put(usb_clk);
	usb_put_hcd(hcd);
	return retval;
}

static int ehci_hcd_s5pv210_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	printk("ehci_hcd_s5pv210_drv_remove\n");
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	s5pv210_stop_ehc();
	clk_put(usb_clk);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ehci_hcd_s5pv210_reinit(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = g_hcd;
	int retval = 0;
	//printk("PKD::%s>>\n",__func__);
	// Remove hcd
	usb_remove_hcd(hcd);
	// Stop ehci
	s5pv210_stop_ehc();
	// start ehci
	s5pv210_start_ehc();
	ehci_hcd_regs_init();
	// Add hcd again
	retval = usb_add_hcd(hcd, pdev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);
	// check for sucess
	if (retval != 0) {
		s5pv210_stop_ehc();
		printk("usb_add_hcd Failed!!! Error=%d\n",retval);
		return retval;
	}
	//printk("PKD::%s Success <<\n",__func__);
	return retval;
}
// PKD@28/11/2010
// Added for reinitialzing some of usb structures.
void ehci_hcd_reinit(void)
{
	ehci_hcd_s5pv210_reinit(g_pdev);
}
EXPORT_SYMBOL(ehci_hcd_reinit);

static struct platform_driver  ehci_hcd_s5pv210_driver = {
	.probe		= ehci_hcd_s5pv210_drv_probe,
	.remove		= ehci_hcd_s5pv210_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.suspend	= ehci_hcd_s5pv210_drv_suspend,
	.resume		= ehci_hcd_s5pv210_drv_resume,
	.driver = {
		//.name = "s5pv210-ehci",
		.name = "s5p-ehci",
		.owner = THIS_MODULE,
	}
};

MODULE_ALIAS("platform:s5p-ehci");
//MODULE_ALIAS("platform:s5pv210-ehci");
