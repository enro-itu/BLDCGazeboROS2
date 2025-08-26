# 0) Hazırlık
Terminali aç:
```bash
sudo apt update
sudo apt install -y git curl wget build-essential software-properties-common gnupg lsb-release
```

# 1) Linux terminal temelleri
- Şuradaki iki kaynağı baştan sona çalış:  
  1) ubuntu başlangıç: https://ubuntu.com/tutorials/command-line-for-beginners#1-overview  
  2) hızlı kopya kâğıdı: https://gist.github.com/bradtraversy/cc180de0edee05075a6139e42d5f28ce  

Minimum hedef: `ls, cd, pwd, mkdir, rm -rf, cp, mv, cat, less, nano/vim, grep, find, top/htop, ps, kill, chmod/chown, apt` komutlarını rahatça kullanmak.

# 2) VS Code (rahat kullanım için)
- Site: https://code.visualstudio.com/docs/setup/linux (sayfanın üstündeki `.deb` linkine tıkla → indir → çift tıkla kur)
- VS Code eklentileri (öneri): “C/C++”, “Python”, “ROS” (ms-iot.vscode-ros), “CMake Tools”, “XML”, “Markdown All in One”, “GitHub Pull Requests”.

# 3) ROS 2 Jazzy kurulumu
Resmî doküman: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html  
- Başlangıç eğitimi (zorunlu kısım):  
  https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html  
  Buradaki **beginner-cli-tools** bölümünü bitir; diğerleri şimdilik opsiyonel.

# 4) Gazebo kurulumu + ros_gz köprüsü
Resmî kurulum: https://gazebosim.org/docs/latest/install_ubuntu/  
Entegre köprü:
```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz
```
Temel Gazebo pratikleri (öğrenme):
- Dünya/robot inşa: https://gazebosim.org/docs/latest/building_robot/
- Hareket ettirme: https://gazebosim.org/docs/latest/moving_robot/

Hızlı test:
```bash
# örnek bir dünya aç (dokümandaki komutlar/örnekler de olur)
gz sim -v 4  # açılan GUI'de örnek world seçebilirsin
```

# 5) GitHub Desktop (isteğe bağlı GUI)
Sayfa: https://github.com/shiftkey/desktop/releases/  
- “Assets” bölümünden **GitHubDesktop-linux-amd64-3.4.9-linux1.deb** → indir → çift tıkla kur.

# 6) Projeye eklenme & projeyi klonlama
- Sonra klonla ve derle:
```bash
# çalışma alanı
mkdir -p ~/bldc_ws/src
cd ~/bldc_ws/src
git clone https://github.com/enro-itu/BLDCGazeboROS2.git
cd ..
# ROS ortamı açık olsun
source /opt/ros/jazzy/setup.bash
# bağımlılıklar (varsa) ve derleme
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
# paket görünüyor mu?
ros2 pkg list | grep -i bldc
```

# 7) Doğrulama checklist (kısa)
- [ ] Terminal temel komutları uygulamalı çalıştım (liste: `ls, cd, mkdir, rm, grep, find, top, ps, chmod, apt`).
- [ ] VS Code kurulu, ROS eklentisi açık.
- [ ] `ros2 doctor` hatasız.
- [ ] `talker`/`listener` örnekleri çalıştı.
- [ ] Gazebo GUI açılıyor (`gz sim`), basit dünya çalışıyor.
- [ ] `ros-jazzy-ros-gz` kurulu.
- [ ] Repo klonlandı, `colcon build` başarıyla tamamlandı.
- [ ] `source install/setup.bash` sonrası paketler listede.

