# ✅ COMPLETE STEP-BY-STEP GUIDE

---

# STEP 1️⃣ — Verify Your Container Is Running

Run:

```bash
docker ps
```

You will see something like:

```
CONTAINER ID   IMAGE        COMMAND       STATUS       NAMES
a1b2c3d4e5f6   ros2_image   "/bin/bash"   Up 3 hours   omx_container
```

Important values:

* **CONTAINER ID** (e.g., `a1b2c3d4e5f6`)
* or **NAME** (e.g., `omx_container`)

You can use either one in later steps.

---

# STEP 2️⃣ — (Optional but Recommended) Clean Temporary Files

Inside your container, if you want to clean:

```bash
docker exec -it omx_container bash
```

Then maybe:

```bash
apt clean
rm -rf /tmp/*
```

This keeps your image smaller.

Exit:

```bash
exit
```

---

# STEP 3️⃣ — Commit the Container to an Image

Now the main step.

Run:

```bash
docker commit <container_name_or_id> <new_image_name>:<tag>
```

Example:

```bash
docker commit omx_container omx_saved:latest
```

Or:

```bash
docker commit a1b2c3d4e5f6 omx_saved:v1
```

Explanation:

| Part          | Meaning                |
| ------------- | ---------------------- |
| omx_container | your running container |
| omx_saved     | name of your new image |
| latest        | version tag            |

After running this, Docker creates a new image from the current state.

---

# STEP 4️⃣ — Verify Image Was Created

Run:

```bash
docker images
```

You should see:

```
REPOSITORY     TAG       IMAGE ID
omx_saved      latest    98fd234abc12
```

Now your work is safe as long as this image exists.

---

# STEP 5️⃣ — Test That It Works

Let’s confirm everything works properly.

Run:

```bash
docker run -it omx_saved:latest bash
```

Inside the container, check your files:

```bash
ls
cd ~/your_workspace
```

If everything is there → success ✅

Exit:

```bash
exit
```

---

# STEP 6️⃣ — (VERY IMPORTANT) Backup the Image to a File

Right now, the image is only stored locally on your system.

If your system crashes or Docker is removed → it is gone.

So export it to a file.

Run:

```bash
docker save -o omx_saved_backup.tar omx_saved:latest
```

This creates:

```
omx_saved_backup.tar
```

This file contains your entire image.

Now you can:

* Copy it to external HDD
* Upload to Google Drive
* Store on another machine
* Keep as permanent backup

This is strongly recommended.

---

# STEP 7️⃣ — How to Restore From Backup (In Future)

If something happens:

1. Install Docker again
2. Restore image:

```bash
docker load -i omx_saved_backup.tar
```

3. Verify:

```bash
docker images
```

4. Run container:

```bash
docker run -it omx_saved:latest bash
```

Everything will be restored.

---

# 🚨 VERY IMPORTANT WARNING

If you created files using mounted volumes like:

```
-v /host/path:/container/path
```

Then those files are NOT stored inside the container.

They are stored on your host system.

To check:

```bash
docker inspect omx_container
```

Look for:

```
"Mounts":
```

If you see mounted directories, those exist outside container.

So make sure you also back up:

* Your host workspace folders
* Any mounted volumes

---

To protect your work:

```bash
docker commit <container> <image_name>
docker save -o omx_saved_backup.tar omx_saved:latest
```

That’s it.

Your full ROS benchmarking environment will be preserved.

---

### Workspace Backup (Your Research Code)

You must also ensure these folders are safe:

```bash
~/robotics/omx_vnc_ws
~/robotics/omx_logs
```

Check:

```bash
ls ~/robotics
```

If they exist → they are already outside Docker.

To be extra safe, you can compress them:

```bash
tar -czvf omx_workspace_backup.tar.gz ~/robotics/omx_vnc_ws
tar -czvf omx_logs_backup.tar.gz ~/robotics/omx_logs
```

---

# 🔬 What Happens If Container Dies?

Nothing bad.

You can recreate it using:

```bash
docker run -d --name omx_vnc \
  -p 6080:80 \
  --security-opt seccomp=unconfined \
  --shm-size=2g \
  -e RESOLUTION=1440x900 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DOMAIN_ID=42 \
  -v ~/robotics/omx_vnc_ws:/home/ubuntu/omx_ws \
  -v ~/robotics/omx_logs:/home/ubuntu/logs \
  -v /mnt/wslg/PulseServer:/mnt/wslg/PulseServer \
  -e PULSE_SERVER=unix:/mnt/wslg/PulseServer \
  --device /dev/snd \
  omx_saved:latest
```

Notice:

👉 Only change is image name:
`ghcr.io/tiryoh/ros2-desktop-vnc:jazzy`
⬇
`omx_saved:latest`

Everything else stays same.

Your workspace will reconnect automatically.

---

# 🎯 Final Safety Checklist For You

You are fully safe if you have:

✅ `omx_saved_backup.tar`
✅ Backup of `~/robotics/omx_vnc_ws`
✅ Backup of `~/robotics/omx_logs`

If all three exist → your entire ROS 2 Jazzy benchmarking environment is permanently protected.

---

# 🔴 One More Important Check

Since you are doing benchmarking work, logs might be stored in:

```
~/.ros/log
```

Check:

```bash
ls -lh ~/.ros
```

If there is a `log` folder there, you may want to back that up too:

```bash
tar -czvf ros_hidden_logs_backup.tar.gz ~/.ros/log
```

---

# 🎯 Current Safety Status

✔ Docker image saved
✔ Docker image exported (21.5GB)
✔ Workspace backed up (60MB)
✔ Logs backed up (empty or minimal)

You are now about 95% protected.

---

# 🏁 Final Suggestion

Now copy all backup files to Windows:

```bash
cp omx_* /mnt/c/Users/<your-windows-username>/Desktop/
```

Or to external drive.

---

