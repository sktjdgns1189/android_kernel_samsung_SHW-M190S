===================================================================================================================== 
Linux v1.01.16 Driver Release Notes for SMSC LAN95XX Family USB 2.0 to Ethernet Controller and USB Hub with Ethernet 
===================================================================================================================== 
 
Contents: 
 
1. Platforms and OS versions supported 
2. Device support 
3. Driver structure and file description 
4. Building and installing the driver 
5. Release history 
6. Testing  
7. Known issues 
8. Driver initialization parameter 
9. Debugging 
 
1. Platforms and kernel versions supported 
------------------------------------------ 
 
    - x86/x64 bit PC 
    - kernel up to 2.6.37
 
2. Device support 
----------------- 
 
This release supports: 
    -LAN9500 (VID = 0x424, PID = 0x9500) USB to Ethernet device 
    -LAN9512/LAN9513/LAN9514(VID = 0x424, PID = 0xEC00) 2/4 port USB hub with LAN9500 
    -LAN9500A(VID = 0x424, PID = 0x9E00) USB to Ethernet device 
 
3. Driver structure and file description 
---------------------------------------- 
 
    This driver is comprised of two modules: 
 
    smscusbnet.ko:      Usbnet interface layer (derived from usbnet.ko in Linux kernel) 
    smsc9500.ko:        lan9500 hardware specific module. 
 
    Source files: 
        smscusbnet.h    -   usbnet header file 
        smscusbnet.c    -   usbnet source file 
        smsc9500.h      -   lan9500 hardware specific header file 
        smsc9500.c      -   lan9500 hardware specific source file 
        ioctl_9500.h    -   ioctl header file definitions 
        cmd9500.c       -   application for useful debugging. 
        version.h       -   Driver version number header file. 
 
4. Building and installing the driver 
------------------------------------- 
    The following instructions work fine for a PC build environment, embedded 
    platforms may need slight build modifications, consult your platform documentation. 
    a. Obtain the kernel source tree for the platform in use and build it. 
    b. build using "./build", the smscusbnet.ko & smsc9500.ko modules are created. 
    c. load both modules in the following sequence:  
        insmod smscusbnet.ko     
        insmod smsc9500.ko 
    d. Plug in the lan9500 device into the USB port. 
    e. Configure the ethernet interface eth<n> in the usual way for an ethernet device. 
 
5.  Release history 
-------------------
    v1.01.16 (01/21/2011)
        - Changed not to check eeprom size during the initialization time. 
    v1.01.15 (01/19/2011)
        - Fixed error checking while reading from internal RAM
    v1.01.14 (01/18/2011)
	- Fixed attach/detach bug.
    v1.01.13 (12/17/2010)
	- Added support for kernel up to 2.6.37
    v1.01.12 (09/15/2010)
	- Fix bug: unable to set mac address (kernel <29)
	- Fix Ethtool speed and auto-negotiation options
    v1.01.11 (09/03/2010)
	- Fix for EEPROM write through ethtool
    v1.01.10 (05/04/2010)
	- EDPD settings for LAN9500A
	- Automdix fix
    v1.01.09 (04/29/2010)
	- Support of changing mac address through ifconfig command
	- Fixed ethtool speed setting options
	- Fixed auto_mdix disable mode options
    v1.01.08 (04/20/2010)
	- Use GFP_DMA flag while allocating memory for usb control transfers
	- Added reset_resume
	- Reading PHY_BSR twice while link check
	- Build issues with kernels up to 2.6.34
	- ethtool -k <interface> gets correct status for rx_csum
	- changed smartdetach to netdetach
    v1.01.07 (03/10/2010)
	- Software workaround for tx checksum offload (1 or 2 byte udp payload) hw bug.
    v1.01.06 (01/26/2010)
        - Added .ndo support for kernels 2.6.29 and up.
    v1.01.03 (10/09/2009)
       - Fixed a configuration problem. If kernel configuration CONFIG_USB_SUSPEND is disabled and
	 autosuspend is enabled, previous version won't transfer any packets.

    v1.01.02 (6/23/2009)
       - Fixed the problem that previous version didn't support kernel without power management enabled.
       - Removed some false EEPROM warning messages.
       - Added smartdetach support when kernel version is less than 2.6.18 for LAN9500A.
       - Fixed a minor memory issue.
    
    v1.01.01 (4/30/2009)
	- Fixed bug that linkdown suspend didn't work on LAN9500A.
	- Fixed unresponsive problem sometimes when smartdetach is enabled.
	- Added new commands WRITE_FILE_TO_EEPROM, READ_EEPROM_TO_FILE, VERIFY_EEPROM_WITH_FILE 
	  in the utility cmd9500.  
	 
    v1.01.00 (4/16/2009)
    - Enable PHY energy detection before entering dynamic suspend. Therefore, driver gets chance to switch 
      suspend state when link changes.
	 
    v1.00.10 (4/06/2009) 
	- Support dynamic suspend for LAN9500 and LAN9500A. 
	- Support smart detach for LAN9500A. 
	- Update statistics counters for LAN9500A. 
	- Add EEPROMless support for LAN9500 and LAN9512. 
	- Add LAN9500A alternative LED configuration. 
		 
    v1.00.09 (3/27/2009) 
        - Add support for separate LEDs for link activity and link status. 
        - Add support that device will suspend when link is down. 
     
    v1.00.08 (3/20/2009) 
        - Add support for LAN9512/9514. 
        - Add support for LAN9500A. 
        - Fix a bug related to ethtool command "ethtool -s ethX autoneg off". 
         
    v1.00.07 (10/20/2008) 
        - Add GET_ERRORs command to cmd9500 to retrieve internal USB error counters. 
        - Enable autosuspend function (only for kernet 2.6.19 and higher) 
          NOTE: in order to use this feature it needs to be enabled in the kernel 
          as well. For this type "echo auto > /sys/bus/usb/devices/5-7/power/level", 
          (where ../5-7/.. is bus number of the device) from the shell. 
        - Expand error recovery for any kind of USB pipe errors. 
        - Fixed compile wanrnings for x64 kernel builds. 
 
    v1.00.06 (internal release) 
        - Support x64 (64 bit PC) kernels builds. 
        - Add error recovery for TXE. 
        - Add cmd9500 application to access lan9500 registers and perform other  
          task of interest (mostly for debugging) 
 
    v1.00.05 (internal release) 
        - Add support for automdix overide strap. 
 
6. Testing  
---------- 
   This is an engineering release.

7. Known Issues 
---------------------------- 
 
    - When using operational_mode=1 (which is not the not the driver's 
      default), we have observed that the device could become unusable  
      if a USB hot swap is performed while stressing active traffic  
      (i.e. ping flood) is ongoing. We have determined this to be  
      caused by a bug in the linux kernel v2.6.25 and higher related to  
      handling of periodic transactions. While the kernel patch currently  
      being proposed (http://marc.info/?t=121837892200003&r=1&w=2) solved  
      that particular problem it introduced others, so it is not a good  
      solution. Once a stable kernel patch for this issue is available 
      this problem should be resolved. 
 
    - On some Linux distributions such as Centos, "pcscd" daemon keeps accessing
      LAN9500/LAN9500A/LAN9512 device for some reasons, which prevents LAN9500 
      device from entering suspend state.  To solve this problem, You can disable
      it by typing "chkconfig pcscd stop".
    
   - To enable autosuspend automatically, add following lines into the file /etc/udev/rules.d/50-udev.rules
		ACTION==¡±add¡±, SUBSYSTEM==¡±usb_device¡±, SYSFS{idVendor}==¡±0424¡±, SYSFS{idProduct}=="9500",RUN+=¡±/bin/sh -c 'echo auto > /sys/$DEVPATH/device/power/level'¡±
		ACTION==¡±add¡±, SUBSYSTEM==¡±usb_device¡±, SYSFS{idVendor}==¡±0424¡±, SYSFS{idProduct}=="9e00",RUN+=¡±/bin/sh -c 'echo auto > /sys/$DEVPATH/device/power/level'¡±
		ACTION==¡±add¡±, SUBSYSTEM==¡±usb_device¡±, SYSFS{idVendor}==¡±0424¡±, SYSFS{idProduct}=="ec00",RUN+=¡±/bin/sh -c 'echo auto > /sys/$DEVPATH/device/power/level'¡±	
      You may have to replace vendor id and product id with your own customized IDs.
      
   - When netdetach is enabled, you have to setup network configuration script to bring up network interface automatically.
   		Suppose smsc9500 Ethernet interface is eth0. Add following lines into /etc/sysconfig/network-scripts/ifcfg-eth0. 
			DEVICE=eth0
			ONBOOT=no
			BOOTPROTO=static
			IPADDR=192.168.1.40   
	
8. Driver Initialization parameters 
----------------------------------- 
 
There are several possible parameters for each driver module 
 
 a. smscusbnet.ko 
        - operational_mode:  
            0:      low_latency mode. Device uses bulk in pipe continuous  
                    reader for ethernet rx. This is the default mode. 
            1:      low power mode. Device uses the interrupt pipe to  
                    detect ethernet rx, only then submits bulk in's to  
                    read rx data. Once no rx data available stops bulk in 
                    submissions. 
        - rx_queue_size:    controls the size of the rx queue 
                    default is 60 
        - tx_queue_size:    controls the size of the tx queue 
                    default is 60 
        - tx_hold_on_completion 
                    default is 1 (enabled). If set the driver will not submit a new 
                    bulk out urb until all previously ones submitted complete 
 
 b. smsc9500.ko 
        - link_mode:  A bit wise field that specifies any combination of 4 or fewer  
                      link speeds 
            0x01:   10HD 
            0x02:   10FD 
            0x04:   100HD 
            0x08:   100FD 
            0x10:   Symmetrical Pause 
            0x20:   Asymmetrical Pause 
            0x40:   Auto Negotiate 
        - auto_mdix:    allows overide to strap in order to control phy's automdix  
                        behavior 
            0:      Disable AMDIX, Straight Cable 
            1:      Disable AMDIX, CrossOver Cable 
            2:      Enable AMDIX 
            3:      Controlled by Strap 
        - mac_addr_hi16 & mac_addr_lo32: allow Ethernet Mac address overide. 
        - scather_gather: enabled kernel S/G support (needed for tx_Csum) 
        - tx_Csum:  Default to 0. Set to 1 to enable hw tx checksum support 
        - rx_Csum:  Default to 0. Set to 1 to enable hw rx checksum support 
        - phy_addr: Default to 1 and using internal phy. Set to appropriate  
                    values if connecting an external phy. 
        - TurboMode: receive mode 
            0:      One receive packet per bulk in transaction 
            1:      Multiple receive packet per bulk in transaction 
        - debug_mode: controls amount of debug verbosity (see debugging below) 
         
        - linkdownsuspend: 
            0:  Disabled. 
            1:  Enabled with normal power savings (recommended for maximum compatibility) 
            2:  Enabled with maximum power savings. 
             
        - LinkActLedCfg: if true, enable hardware driven LED configuration on LAN9500A. Only  
        		 applies to LAN9500A.  
        - LinkLedOnGpio: Selects the GPIO number (in the range 0 to 10) use for link status  
                         indication. If this parameter is not specified, the LED handling  
                         is the original hardware default for combined link/activity pin. 
        - LinkLedBufType: if enabled (will be only if LinkLedOnGpio is used), set GPIO port  
                         buffer type as open-drain. Otherwise, it is push/pull.  
        - LinkLedPolarity: if enabled (will be only if LinkLedOnGpio is used), GPIO output  
                         will be low active. Otherwise, will be high active. 
               
	- dynamicsuspend: if enabled, suspend device automatically if no 
			traffic for 3 seconds, then waking up device if any Tx/Rx traffic.  
						   
	- netdetach: if enabled, detach device from USB bus if link is down, then re-
			attaching device to USB bus if link is back.						   
     
9. Debugging 
------------ 
     a. You can use the debug_mode insmod parameter to control printk verbosity  
        of this driver. See DBG_xxx #defines in smsc9500.h for details. 
 
     b. The cmd9500.c provides several useful commands that can facilitate runtime  
        debugging by allowing register manipulations and/or giving access to internal 
        driver data. See it's usage page DisplayUsage() for details. 

