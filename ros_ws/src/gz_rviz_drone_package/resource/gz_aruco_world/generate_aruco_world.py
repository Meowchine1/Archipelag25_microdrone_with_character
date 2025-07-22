import os
import cv2
import cv2.aruco as aruco

# Настройки
MARKER_SIZE_M = 0.33
MARKER_STEP_M = 1.0
GRID_SIZE = 7

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(BASE_DIR, "models")
WORLDS_DIR = os.path.join(BASE_DIR, "worlds")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

def create_aruco_model(idx):
    model_name = f"aruco_{idx}"
    model_path = os.path.join(MODELS_DIR, model_name)
    tex_dir = os.path.join(model_path, "materials", "textures")

    print(f"[INFO] Генерация модели ArUco #{idx} в {model_path}")
    os.makedirs(tex_dir, exist_ok=True)

    try:
        marker_img = aruco.generateImageMarker(aruco_dict, idx, 200)
        marker_path = os.path.join(tex_dir, "aruco.png")
        cv2.imwrite(marker_path, marker_img)
        print(f"[OK] Изображение метки сохранено: {marker_path}")
    except Exception as e:
        print(f"[ERROR] Ошибка генерации изображения метки {idx}: {e}")

    config_path = os.path.join(model_path, "model.config")
    sdf_path = os.path.join(model_path, "model.sdf")

    try:
        with open(config_path, "w") as f:
            f.write(f"""<?xml version="1.0" ?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Script</name></author>
  <description>ArUco marker {idx}</description>
</model>
""")
        print(f"[OK] model.config записан: {config_path}")
    except Exception as e:
        print(f"[ERROR] Ошибка записи model.config для {model_name}: {e}")

    try:
        with open(sdf_path, "w") as f:
            f.write(f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{MARKER_SIZE_M} {MARKER_SIZE_M}</size>
          </plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>model://{model_name}/materials/textures/aruco.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{MARKER_SIZE_M} {MARKER_SIZE_M}</size>
          </plane>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
""")
        print(f"[OK] model.sdf записан: {sdf_path}")
    except Exception as e:
        print(f"[ERROR] Ошибка записи model.sdf для {model_name}: {e}")

def create_sphere_model(name, rgba):
    path = os.path.join(MODELS_DIR, name)
    print(f"[INFO] Создание сферы {name} в {path}")
    os.makedirs(path, exist_ok=True)

    try:
        with open(os.path.join(path, "model.config"), "w") as f:
            f.write(f"""<?xml version="1.0" ?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Script</name></author>
  <description>{name} sphere</description>
</model>
""")
        print(f"[OK] {name}: model.config записан")
    except Exception as e:
        print(f"[ERROR] Ошибка записи model.config для сферы {name}: {e}")

    try:
        with open(os.path.join(path, "model.sdf"), "w") as f:
            f.write(f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")
        print(f"[OK] {name}: model.sdf записан")
    except Exception as e:
        print(f"[ERROR] Ошибка записи model.sdf для сферы {name}: {e}")

def generate_world():
    os.makedirs(WORLDS_DIR, exist_ok=True)
    world_path = os.path.join(WORLDS_DIR, "aruco_field.sdf")
    print(f"[INFO] Генерация мира: {world_path}")

    try:
        with open(world_path, "w") as f:
            f.write("""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="aruco_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
""")
            for row in range(GRID_SIZE):
                for col in range(GRID_SIZE):
                    idx = row * GRID_SIZE + col
                    x = col * MARKER_STEP_M
                    y = row * MARKER_STEP_M
                    f.write(f"""    <include>
      <uri>model://aruco_{idx}</uri>
      <pose>{x} {y} 0 0 0 0</pose>
    </include>
""")

            # Шары
            spheres = {
                "sphere_red":    ("3.0 3.0 0.1", "1 0 0 1"),
                "sphere_green":  ("1.5 5.5 0.1", "0 1 0 1"),
                "sphere_blue":   ("5.5 1.5 0.1", "0 0 1 1"),
                "sphere_yellow": ("5.0 5.0 0.1", "1 1 0 1")
            }
            for name, (pose, rgba) in spheres.items():
                f.write(f"""    <include>
      <uri>model://{name}</uri>
      <pose>{pose} 0 0 0</pose>
    </include>
""")

            # Дроны
            f.write("""    <include>
      <uri>model://x500_mono_cam</uri>
      <name>micro_drone_0</name>
      <pose>0 0 0.3 0 0 0</pose>
    </include>
    <include>
      <uri>model://x500_mono_cam</uri>
      <name>micro_drone_1</name>
      <pose>6 6 0.3 0 0 0</pose>
    </include>
  </world>
</sdf>
""")
        print(f"[OK] Мир сгенерирован: {world_path}")
    except Exception as e:
        print(f"[ERROR] Ошибка генерации мира: {e}")

# --- Главный запуск ---
if __name__ == "__main__":
    print("[START] Генерация ArUco-моделей и мира Gazebo...")

    for i in range(GRID_SIZE * GRID_SIZE):
        create_aruco_model(i)

    create_sphere_model("sphere_red", "1 0 0 1")
    create_sphere_model("sphere_green", "0 1 0 1")
    create_sphere_model("sphere_blue", "0 0 1 1")
    create_sphere_model("sphere_yellow", "1 1 0 1")

    generate_world()

    print("[DONE] Всё готово.")
