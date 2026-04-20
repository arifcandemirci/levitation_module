## Levitation Plugin - Algoritma Detayları

Bu doküman `plugins/levitation_plugin.cpp` içindeki kontrol algoritmasının matematiksel ve uygulama detaylarını açıklar. Amaç: tuning, test ve genişletme için net bir teknik referans sağlamaktır.

### Temel değişkenler ve notasyon
- z_b: robotun CoG Z pozisyonu (body_pose.Pos().Z())
- z_p_center: plakanın merkez Z pozisyonu
- plate_thickness: plaka kalınlığı
- plate_bottom_z = z_p_center - 0.5 * plate_thickness
- target_gap (g): plaka ile robot topu arasındaki hedef boşluk
- h_top: module_top_offset (robot üst yüzünden CoG'ye offset)
- target_body_z = plate_bottom_z - g - h_top
- m: toplam kütle (total_mass_)
- g_acc: yerçekimi büyüklüğü (gravity_mag_)

### Kontrol hedefleri
- Dikey (Z): robotun z pozisyonunu target_body_z etrafında tutmak
- Planar (X/Y): teleop girdilerine göre düzlemde hareket sağlamak ve hızı sönümlendirmek
- Rotasyon: başlangıç rotasyonunu korumak için restore torque uygulamak

### Dikey kontrol (Z)

Formül:

```
z_error = target_body_z - z_b
fz = m * g_acc + z_kp * z_error - z_kd * linear_vel.z
fz = clamp(fz, 0.0, z_force_max)
```

Açıklama:
- `m * g_acc`: ağırlığı dengeleme terimi; sistem nominalde ağırlığı dengeler.
- `z_kp * z_error`: pozisyon düzeltmesi (proportional).
- `- z_kd * linear_vel.z`: hızdan kaynaklı salınımları sönümlendiren terim (damping).
- Clamp ile kuvvet sınırlandırılır (0 .. z_force_max).

Numerik öneriler:
- Önce z_force_max ve spawn_z değerlerini ayarlayıp robotun hedefe erişebildiğinden emin olun.
- z_kp yüksekse z_kd ile birlikte kritik sönümlemeye yakın ayarlayın.
- Fizik adım büyüklüğü (`max_step_size`) küçük ve stabil olmalıdır.

### Planar kontrol (X/Y)

Formül (X ekseni örneği):

```
cmd_x = clamp(cmd.linear.x, -1, 1)
fx = planar_force_gain * cmd_x - planar_damping * linear_vel.x
fx = clamp(fx, -planar_force_max, planar_force_max)
```

- `planar_force_gain` teleop komutunu fiziksel kuvvete çevirir.
- `planar_damping` mevcut hızı sönümlendirir.
- Teleop komutları [-1,1] aralığında beklenir.

### Rotasyon restore (Torque)

Adımlar:
1. `locked_orientation_` Load sırasında kaydedilir (başlangıç rotasyonu).
2. Her güncellemede hata kuaternionu: `q_err = locked_orientation_ * current_rot.Inverse()`
3. `angle_err = q_err.Euler()` (küçük açı varsayımı)
4. `torque = rot_kp * angle_err - rot_kd * angular_vel`

Notlar:
- Euler dönüşleri büyük açılarda sorun yaratabilir; geniş dönüşler için quaternion->axis-angle kullanın.

### Pseudocode (OnUpdate)

```
if not links: return
now = world->SimTime()
dt = now - last_update_time
if dt <= 0: return

body_pose = body_link->WorldCoGPose()
linear_vel = body_link->WorldLinearVel()
angular_vel = body_link->WorldAngularVel()

plate_center_z = plate_link->WorldCoGPose().Pos().Z()
plate_bottom_z = plate_center_z - 0.5 * plate_thickness
target_body_z = plate_bottom_z - target_gap - effective_module_top_offset

z_error = target_body_z - body_pose.Pos().Z()
fz = m*g_acc + z_kp*z_error - z_kd*linear_vel.Z()
fz = clamp(fz, 0, z_force_max)

copy cmd under mutex
fx = planar_force_gain*clamp(cmd.x,-1,1) - planar_damping*linear_vel.X()
fy = planar_force_gain*clamp(cmd.y,-1,1) - planar_damping*linear_vel.Y()
fx,fy = clamp to planar_force_max

AddForce(fx,fy,fz)

q_err = locked_orientation * current_rot.Inverse()
angle_err = q_err.Euler()
torque = rot_kp*angle_err - rot_kd*angular_vel
AddTorque(torque)
```

### Sayısal örnek
- Parametreler örneği: m=0.5 kg, g_acc=9.81, z_kp=180, z_kd=30, plate_bottom_z=1.98, body_z=1.95, linear_vel.z=-0.02

Hesap:
- target_body_z = 1.98 - 0.02 - 0.0025 = 1.9575
- z_error = 1.9575 - 1.95 = 0.0075
- fz = 0.5*9.81 + 180*0.0075 - 30*(-0.02) = 6.855 N

Yorum: ağırlık ~4.9 N; kontrol ek ~1.95 N ile hedefe doğru düzeltme sağlar.

### Kararlılık ve tuning ipuçları
- z_kp/z_kd: önce z_force_max'i ayarlayın, sonra küçük adımlarla kp/kd artışı yapın.
- Planar gain/damping: yüksek gain -> hızlı tepki ama overshoot; damping ile dengeleyin.
- Orientation: küçük açı yaklaşımları için Euler uygundur; büyük sapmalar quaternion tabanlı düzeltme gerektirir.

### Test önerileri
- Unit test: fz hesap fonksiyonunu izole edip edge case'leri test et.
- Integration: headless Gazebo ile spawn ardından belirli komutlar yayınla, z hatasını izleyin.

### Geliştirme önerileri
- Telemetry publisher: `rclcpp::Publisher` ile `z_error`, `applied_force`, `last_cmd` yayınla.
- Runtime parametreler: rclcpp param veya dynamic param sistemi ile gain'leri değiştir.
- Model B genişletmesi: dört-pad kuvvet modeline genişleyerek moment kontrolünü detaylandır.

---

Bu belge algoritmanın mantığını ve uygulama detaylarını içerir. İsterseniz şimdi plugin'e telemetry publisher ekleyen küçük bir patch uygulayıp derleyeyim ve kısa bir doğrulama çalıştırayım.
