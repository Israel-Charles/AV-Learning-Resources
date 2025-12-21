# Lab 0: Installing Ubuntu 22.04 on VMware virtual machine

**Platforms:** Windows, macOS (Intel chips), macOS (Apple Silicon chips)

---

## Objective

In this lab, you will:

1. Install **VMware** virtual machine on your computer
2. Create a **virtual machine (VM)** environment
3. Install **Ubuntu 22.04**
4. Verify a working Ubuntu desktop environment

---

## Requirements

* A computer running **Windows**, **macOS (Intel)**, or **macOS (Apple Silicon)**
* At least **8 GB RAM recommended**
* At least **40 GB free disk space**
* Internet connection

---

## Step 1: Install VMware (Differs by Platform)

### Windows

* VMware product: **VMware Workstation Pro (Free for personal use)**
* Download from:
  [https://www.techpowerup.com/download/vmware-workstation-pro/](https://www.techpowerup.com/download/vmware-workstation-pro/)
  * Click on the `Download` button for Windows (This tutorial is tested on the `25H2` version, but feel free to try a newer version if available)
  * Select a server to download it from, and the download should start automatically (the server closest to you is usually a good choice)

**Install steps:**

1. Run the downloaded `.exe`
2. Click **Next**
3. Accept license agreement
4. Use default options
5. Click **Install**
6. Restart if prompted

---

### macOS (Intel & Apple Silicon)

* VMware product: **VMware Fusion Pro (Free for personal use)**
* Download from:
  [https://www.techspot.com/downloads/2755-vmware-fusion-mac.html](https://www.techspot.com/downloads/2755-vmware-fusion-mac.html)
  * From the download options, click on a version, and the download should start automatically (This tutorial is tested on the `25H2` version, but feel free to try a newer version if available)

**Install steps:**

1. Open the downloaded `.dmg`
2. Double click **VMware Fusion** or drag it to **Applications** (depends on installer)
3. Launch VMware Fusion if not automatically launched
4. Approve permissions in **System Settings → Privacy & Security → Accessibility**
5. Restart if requested

> VMware Fusion automatically installs the correct version for **Intel** or **Apple Silicon** Macs.

---

## Step 2: Download Ubuntu 22.04 (Critical Differences)

### Windows & macOS Intel

Download **Ubuntu 22.04 Desktop (amd64)**:
[https://releases.ubuntu.com/jammy/](https://releases.ubuntu.com/jammy/)

On the `Desktop image` section, click on `64-bit PC (AMD64) desktop image`, and the download should start automatically

File name example:

```
ubuntu-22.04.x-desktop-amd64.iso
```

This is the **official Ubuntu Desktop ISO**.

---

### macOS Apple Silicon Chips

> **Important:** Ubuntu does **not** provide an official Desktop ARM ISO.

Instead, download **Ubuntu 22.04 Server (ARM)**:
[https://cdimage.ubuntu.com/releases/jammy/release/](https://cdimage.ubuntu.com/releases/jammy/release/)

On the `Server install image` section, click on `64-bit ARM (ARMv8/AArch64) server install image`, and the download should start automatically

File name example:

```
ubuntu-22.04.x-live-server-arm64.iso
```

> You will install the **desktop later**.

---

## Step 3: Create the Virtual Machine

### Windows (VMware Workstation Pro)

1. Open **VMware Workstation Pro**
2. Click **Create a New Virtual Machine**
3. Select **Installer disc image file (iso)**
4. Browse to the Ubuntu ISO file you downloaded earlier
5. Click **Next**
6. Follow the recommended setting options
7. Name the VM (e.g., `Ubuntu 22.04`)
8. Disk size:

   * Recommended: **20–40 GB**
9. Click **Finish**

---

### macOS (VMware Fusion – Intel & Apple Silicon)

1. Open **VMware Fusion Pro**
2. Click **Create New** if prompted
3. Select **Install from disc or image** then press `Continue`
4. Drag the Ubuntu ISO file recently downloaded or click on `Use another disc or disc image...` to choose the Ubuntu ISO file downloaded
5. Click **Continue**
6. Follow the recommended setting options (Disk size: 20-40 GB)
7. Finish setup

---

## Step 4: Install Ubuntu

### Windows & macOS Intel (Desktop Installer)

1. Press `Ok` on the annoucement pop up notifications
2. VM boots into Ubuntu installer
3. Select **Try or Install Ubuntu** if prompted
4. Choose language
5. Keyboard layout → **Continue**
6. Updates & other software:

   * **Normal installation**
   * **Download updates while installing Ubuntu**
   * **Install third-party software ...**
7. Installation type:

   * **Erase disk and install Ubuntu** (This affects only the space you had given the virtual machine, not your entire computer)
8. Click on `Install Now` followed by `Continue` if promted to `Write the changes to disks?`
9. Select time zone
10. Create user account
11. Click **Install**
12. Restart when prompted

> **When asked to upgrade Ubuntu, such as going from the 22.04 installed to 24.04, it is imperative that you do not upgrade**
---

### macOS Apple Silicon (Server Installer)

1. Select **Try or Install Ubuntu** if prompted (use the arrow keys to change options and press `Enter` to select options)
2. VM boots into **text-based Ubuntu Server installer**
3. Choose:

   * Language
   * Keyboard layout
4. If prompted to update the installer, feel free to do so
5. Choose the base for the installation:
     * **Ubuntu Server**
     * **Search for third-party drivers**
7. Network:

   * Accept default (automatic)
8. Storage:

   * **Use entire disk** (This affects only the space you had given the virtual machine, not your entire computer)
   * Accept defaults
9. Click on `Continue` if prompted about destructive action on the disk (This affects only the space you had given the virtual machine, not your entire computer)
10. Profile setup:

   * Username
   * Password
12. Skip Ubuntu Pro
13. SSH:

   * Optional (can skip)
14. Can skip any additional options
15. Finish installation (might take a while)
16. Reboot
17. Press `Enter` if prompted to remove installation media
18. Log in at the terminal

At this point, **no desktop is installed yet**.

---

## Step 5: Install Ubuntu Desktop (Apple Silicon Only)

After logging in, run:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ubuntu-desktop -y
```
Press enter if asked `Which services should be restarted`
> This may take several minutes.

When finished:

```bash
sudo reboot
```

After reboot, the **Ubuntu 22.04 Desktop (GNOME)** will load.

> **When asked to upgrade Ubuntu, such as going from the 22.04 installed to 24.04, it is imperative that you do not upgrade**

---

## Step 6: Install VMware Tools (All Platforms)

VMware Tools improves:

* Display resolution
* Mouse integration
* Performance

### Inside the Ubuntu VM

In Ubuntu, open a terminal (go on apps and look for `Terminal` and run:

```bash
sudo apt install open-vm-tools-desktop -y
sudo reboot
```

---

## Step 7: Verification

After reboot, confirm:

* Ubuntu desktop loads successfully
* Screen resizes automatically
* Mouse moves smoothly in and out of the VM

---

## Platform Summary (Verified)

| Platform            | Ubuntu Method                                |
| ------------------- | -------------------------------------------- |
| Windows             | Ubuntu 22.04 Desktop (amd64 ISO)             |
| macOS Intel         | Ubuntu 22.04 Desktop (amd64 ISO)             |
| macOS Apple Silicon | Ubuntu 22.04 Server (ARM) + `ubuntu-desktop` |

---

## Notes

* Installing Ubuntu in a VM **does not affect your real operating system**
* Apple Silicon Macs require **ARM-based Linux**
* Using Ubuntu Server + desktop is **fully supported and standard practice**
