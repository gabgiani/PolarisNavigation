# Polaris Navigation Plan

## 1. Objetivo y Alcance
- **Objetivo:** planear y ejecutar trayectorias seguras y dinámicamente factibles de A→B con evasión de obstáculos en 2D/3D, integrable con PX4 y MAVROS para facilitar la migración a un dron real. El simulador principal será AirSim.
- **Alcance inicial (Fase I):** entorno conocido con obstáculos estáticos.
- **Evolución (Fase II):** entorno parcialmente desconocido, obstáculos dinámicos y replaneamiento en línea.

## 2. Arquitectura de Módulos (ROS 2)
- **Global Planner (GP):** genera rutas discretas (waypoints) a partir del mapa/ocupación.
- **Local Planner (LP):** ajusta la trayectoria ante obstáculos cercanos de forma reactiva.
- **Trajectory Generator (TG):** convierte los waypoints en curvas suaves y factibles (mínimo jerk/snap).
- **Feasibility Checker (FC):** valida límites dinámicos (velocidad/aceleración/jerk), clearance y colisiones.
- **Controller Bridge (CB):** publica setpoints a PX4 (posición/velocidad/actitud) vía MAVLink/MAVROS u Offboard.
- **World Model (WM):** mantiene el mapa estático y la capa de costos dinámica proveniente de sensores.
- **Supervisor (SUP):** gestiona la misión, dispara replaneos y monitoriza KPIs.

### Temas ROS 2 sugeridos
- `/mission/goal` (`geometry_msgs/PoseStamped`)
- `/planner/global_path` (`nav_msgs/Path`)
- `/planner/local_path` (`nav_msgs/Path`)
- `/planner/trajectory` (`traj_msgs/PolynomialTrajectory` o mensaje propio)
- `/world/costmap` (`nav_msgs/OccupancyGrid`) + capa 3D opcional (Octomap/Voxel)
- `/offboard/setpoint_*` (`mavros_msgs/...`)

## 3. Algoritmos por Componente

### 3.1 Global Planner (Fase I → II)
- **Fase I (2D/2.5D):** A* en grid (heurística euclídea/octile) con suavizado posterior.
- **Fase II (3D/continuo):** RRT* o BIT* (OMPL) con costo multicriterio (longitud + riesgo + consumo).
- **Extensiones:** D* Lite/Anytime D* para cambios locales en el mapa.

### 3.2 Local Planner
- **Inicio:** DWA o VFH (simple y rápido).
- **Evolución:** MPC (model predictive control) para respetar la dinámica del UAV y restricciones.
- **Librerías:** acados, CasADi (usar para ajuste offline si no hay aceleración en vuelo).
- **Política de prioridad:** el LP manda si hay conflicto inmediato; el GP replanea si el desvío supera un umbral.

### 3.3 Generador de Trayectorias
- **Método base:** polinomios mínimos de jerk/snap por segmentos (Mellinger & Kumar).
- **Alternativas:** splines cúbicos/B-splines con continuidad C³.
- **Salida:** `pos(t)`, `vel(t)`, `acc(t)` a 50–100 Hz.

### 3.4 Colisión y Factibilidad
- **Chequeo de colisión 2D:** inflado por el radio del dron en el costmap.
- **Chequeo de colisión 3D:** FCL (Flexible Collision Library) sobre voxel/Octomap/Voxblox.
- **Dinámica:** límites de `v_max`, `a_max`, `j_max`, `yaw_rate`, `banking angle`.
- **Clearance dinámica:** margen dependiente de la velocidad (mayor velocidad ⇒ mayor margen).

## 4. Representación del Mundo (WM)
- **Fase I:** `OccupancyGrid` 2D + capa de costos con inflación.
- **Fase II:** voxel/OctoMap o Voxblox (ESDF) para planificación 3D suave.
- **Percepción:** fusión de Lidar/Depth/OAK-D (VIO) + IMU.
- **SLAM sugerido:** ORB-SLAM3 (mono/estéreo-inercial) u OpenVSLAM; para malla/ESDF: Voxblox.

## 5. Integración con PX4
- **Modo:** Offboard vía MAVROS/microRTPS.
- **Control:** envío de setpoints en posición/velocidad + yaw/yaw_rate; fallback a altitude hold/RTL.
- **Frecuencia:** 20–50 Hz de setpoints estables (reloj estable ⇒ bajo jitter).
- **Seguridad:** geofence, pérdida de señal, failsafe de batería.

## 6. Tooling y Librerías Recomendadas
- OMPL (Global planner: RRT*, BIT*, PRM*).
- Eigen (álgebra).
- FCL (colisiones).
- CasADi / acados (MPC y optimización).
- OctoMap / Voxblox (mapa 3D).
- PCL/OpenCV (percepción).
- `rclcpp`, `nav_msgs`, `geometry_msgs` (ROS 2).

## 7. Simulación y Datos
- **Simulación:**
  - Gazebo (Ignition) para dinámica UAV clásica.
  - AirSim si se requiere realismo visual para visión/SLAM.
- **Datasets:** EuRoC MAV (VI-SLAM), TUM RGB-D, secuencias propias con OAK-D.

## 8. Fases de Entrega y Criterios de Aceptación

### Fase I — Entorno conocido, estático (4–6 semanas)
1. **GP A*** en grid 2D  
   *DoD:* genera path sin colisiones con longitud ≤ 1.2× el óptimo en mapas de prueba.
2. **TG polinómico mínimo jerk**  
   *DoD:* continuidad C³, límites de velocidad/aceleración respetados, muestreo estable a 50 Hz.
3. **CB a PX4 + vuelo en SITL**  
   *DoD:* seguimiento con error medio < 0.5 m y pico < 1.2 m en circuito simple.
4. **SUP + reintentos**  
   *DoD:* sin intervención del LP, alcanza el objetivo con ≥ 95% de éxito en 20 corridas.

### Fase II — Obstáculos dinámicos + replaneo (6–10 semanas)
1. **LP DWA/VFH**  
   *DoD:* evita intrusos emergentes a ≥ 2 m con ≤ 10% de abortos.
2. **Replaneo (D* Lite o A* incremental)**  
   *DoD:* latencia < 200 ms en mapas medianos; ratio de éxito ≥ 90%.
3. **Opcional: MPC local**  
   *DoD:* mejora ≥ 20% la suavidad (jerk promedio) y ≤ 10% de tiempo adicional vs. DWA.

## 9. Métricas (KPIs)
- **Seguridad:** colisiones = 0; clearance mínima ≥ margen configurado.
- **Desempeño:** tiempo de misión, longitud de ruta, energía estimada.
- **Calidad:** jerk medio, suavidad de yaw, overshoot.
- **Robustez:** éxito en N escenarios (≥ 50 corridas Monte Carlo), MTBF de misión.
- **Cómputo:** uso de CPU/GPU, latencias (planner/tg/control loop).

## 10. Interfaces y Mensajes
- **Entradas:**
  - `/mission/goal` (`PoseStamped`)
  - `/world/costmap` (`OccupancyGrid`) + `/world/esdf` opcional
  - `/odom` (`nav_msgs/Odometry`), `/imu`
- **Salidas:**
  - `/planner/global_path` (`Path`)
  - `/planner/local_path` (`Path`)
  - `/planner/trajectory` (mensaje propio con coeficientes polinomiales)
  - `/offboard/setpoint_position` o `/offboard/setpoint_velocity` (MAVROS)

## 11. Gestión de Riesgos
- Desfase de reloj: usar `use_sim_time`/NTP; timeouts estrictos en Offboard.
- Mapas inconsistentes: invalidación por timestamp y confianza (probabilístico).
- Ruido/latencia de sensores: filtros (EKF/UKF), rechazo de outliers, compuertas de Mahalanobis.
- Fallas de replaneo: zonas seguras/hover, RTL o aterrizaje controlado.

## 12. Referencias Técnicas
- Mellinger & Kumar, *Minimum Snap Trajectory Generation for Quadrotors*.
- LaValle, *Planning Algorithms* (fundamentos A*, RRT).
- Documentación de OMPL (RRT*, BIT*).
- Voxblox (ESDF para UAV).
- PX4 Offboard & MAVROS (setpoints y modos).
