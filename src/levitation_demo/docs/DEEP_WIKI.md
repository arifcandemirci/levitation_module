## Levitation Demo - DeepWiki

Bu belge, depo içindeki `levitation_demo` paketinin kapsamlı teknik rehberidir. Amaç: geliştiricilerin hızlıca anlaması, çalıştırması, tuninglemesi ve gerektiğinde genişletmesi için tüm önemli detayları merkezi bir yerde sunmaktır.

## 1. Yönetici Özeti ve Misyon

### Kısa Özet
- Bu paket, Gazebo Classic ve ROS 2 (Foxy) kullanılarak bir "askıda tutulan" (levitation) mobil modülün simülasyonunu sağlar. Ana araç: bir Gazebo ModelPlugin (`levitation_plugin`) ile robot üzerinde sanal kuvvetler uygulamak.

### Hangi problemi çözüyor?
- Bir cismin bir üst plaka altında belirli bir açıklıkta (gap) asılı kalması ve düzlemde komut ile hareket ettirilmesi. Gerçek manyetik levitasyon sistemlerinin fikir prototiplerini hızlıca denemek için uygun.

### Teknolojiler / Kütüphaneler

| Katman | Teknoloji |
|---|---|
| Simülasyon | Gazebo Classic (ODE physics) |
| Robot middleware | ROS 2 Foxy (rclcpp, geometry_msgs) |
| Build | ament_cmake |
| Model tanımı | URDF (gazebo plugin block) |
| Launch | ROS2 launch (Python) |


## 2. Mimari Yapı ve Bileşen Analizi

### Blok Diyagramı (metinsel)

- Gazebo World (`levitation_demo.world`) — static `copper_plate` içerir.
- Robot (URDF `levitation_robot.urdf`) — robot_description yayımlanır, `spawn_entity.py` ile spawn edilir.
- LevitationPlugin (`liblevitation_plugin.so`, kaynak: `plugins/levitation_plugin.cpp`) — ModelPlugin: world/model/link referansları, ROS node, subscriber ve update callback.
- Teleop (Python `teleop_force_keyboard.py`) — `/levitation/cmd_vel` publisher.

İletişim: Teleop -> `/levitation/cmd_vel` -> LevitationPlugin -> Gazebo AddForce/AddTorque

### Klasör Yapısı ve Amaçları

- `plugins/levitation_plugin.cpp` — Kontrol mantığı (Load, OnUpdate, OnCmdVel). Kritik dosya.
- `urdf/levitation_robot.urdf` — Robot model, plugin param block, sensör tanımları (camera, imu).
- `worlds/levitation_demo.world` — Dünya, gravity, copper_plate pozisyonu.
- `launch/levitation_demo.launch.py` — Gazebo başlatma, robot_state_publisher, spawn_entity wrapper.
- `scripts/teleop_force_keyboard.py` — TTY teleop node: `/levitation/cmd_vel` yayıncı.
- `CMakeLists.txt`, `package.xml` — Derleme ve bağımlılıklar.


## 3. Veri Akışı ve Mantıksal İşleyiş

### Başlatma (entry point)
- Komut: `ros2 launch levitation_demo levitation_demo.launch.py` (launch içinde gazebo include, robot_state_publisher, spawn_entity çağrılır.)

### Runtime akış (kısa)
1. Gazebo world yüklenir (copper_plate oluşturulur).
2. Robot URDF spawn edilir; plugin yüklenir (`liblevitation_plugin.so`).
3. Plugin `Load()` içinde SDF parametreleri okur, rclcpp node oluşturur, `/levitation/cmd_vel` aboneliğini başlatır, world update callback'e bağlanır.
4. Kullanıcı veya başka bir node `/levitation/cmd_vel` gönderir.
5. `OnUpdate()` her sim adımında: pozisyon/velocity okunur, z/planar/rotation kuvvetleri hesaplanır, `AddForce/AddTorque` ile fiziğe uygulanır.

### Önemli Topicler ve Mesajlar
- `/levitation/cmd_vel` — geometry_msgs/Twist (linear.x, linear.y) — Kontrol girdisi.
- IMU ve Camera sensor topicleri — URDF içindeki plugin konfigürasyonlarına bağlı.


## 4. Teknik Detaylar ve Bağımlılıklar

### Kritik Sınıflar / Fonksiyonlar
- `class LevitationPlugin : public ModelPlugin` — `plugins/levitation_plugin.cpp`
  - `Load()` — param parse, links model çözümü, rclcpp init, subscriber kurulum, executor thread
  - `OnCmdVel(...)` — `last_cmd_` güncellemesi (mutex ile korunur)
  - `OnUpdate()` — PD benzeri dikey kontrol, planar kuvvet hesapları, orientation restore (AddForce/AddTorque)

### Önemli parametreler (default değerler)
- `target_gap` = 0.02 m, `z_kp`=180, `z_kd`=30, `z_force_max`=120 N
- `planar_force_gain`=8.0, `planar_damping`=6.0, `planar_force_max`=12.0
- `rot_kp`=80.0, `rot_kd`=18.0

### Bağımlılıklar (build/runtime)
- Build: `gazebo_dev`, `rclcpp`, `geometry_msgs`, `ament_cmake`
- Runtime: `gazebo_ros`, `robot_state_publisher`

### Donanım entegrasyonu
- Repo gerçek MCU/drivers içermez; plugin sanal kuvvet uygular. Gerçek donanıma geçişte kontrol mantığını ROS node'a taşıyarak sürücü katmanına bağlamak gerekir.


## 5. Geliştirici Notları ve Tuning

### Hızlı çalışma adımları
```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select levitation_demo
source install/setup.bash
ros2 launch levitation_demo levitation_demo.launch.py

# Ayrı terminal:
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run levitation_demo teleop_force_keyboard.py
```

### Potansiyel darboğazlar
- Çok agresif `z_kp` veya yetersiz `max_step_size` -> osilasyon
- Plugin ve Gazebo update döngüsünde ağır hesaplar -> real-time performans düşer
- Teleop TTY gereksinimi (arka planda çalışmaz)

### Önerilen küçük geliştirmeler
- Plugin'den telemetry yayınlama (applied_force, z_error) — debugging için faydalı
- Parametrelerin runtime değiştirilebilmesi (ROS param, dynamic reconfigure benzeri)
- Kontrol mantığını ayrı ROS node'a çıkarıp plugin'i minimal köprü haline getirme


## Dosya Referansları
- `plugins/levitation_plugin.cpp` — kontrol mantığı
- `urdf/levitation_robot.urdf` — model + plugin param block
- `worlds/levitation_demo.world` — copper_plate
- `launch/levitation_demo.launch.py` — başlatma
- `scripts/teleop_force_keyboard.py` — teleop

---

İleri adım ister misiniz? (1) bunu repo'ya `docs/DEEP_WIKI.md` olarak kaydettim; (2) ayrıca algoritma detayları için ayrı bir doküman ekledim: `ALGORITHM_DETAILS.md`.
