# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (nécessaire pour projection='3d')

def lancer_simulation_3d():
    # ---------
    # PARAMÈTRES DE CONFIGURATION
    # ---------
    perte_signal_active = True
    manoeuvre_esquive_active = True
    manoeuvre_zigzag_active = True

    dt = 0.1
    temps = 800

    std_accel = 0.05
    std_gyro = 0.01
    std_ir = 0.01  # bruit angulaire du senseur (azimut ET élévation)

    N_gain = 4.0

    # CONTRAINTES PHYSIQUES ET AÉRODYNAMIQUES
    G_LIMIT = 20.0
    ACCEL_GRAVITE = 9.81
    MAX_LATERAL_ACCEL = G_LIMIT * ACCEL_GRAVITE
    MAX_SLEW_RATE_OMEGA = 5.0  # rad/s max par axe (lacet, tangage)

    # Paramètres aérodynamiques
    Masse = 15.0
    Rho = 1.225
    S_ref = 0.015
    Cx = 0.25

    # ---------
    # VÉRITÉ TERRAIN : États initiaux réels
    # ---------
    x_t, y_t, z_t = 30.0, 120.0, 80.0
    vx_t, vy_t, vz_t = 1.5, -0.2, 0.3

    x_s, y_s, z_s = 0.0, 0.0, 0.0

    dx0, dy0, dz0 = x_t - x_s, y_t - y_s, z_t - z_s
    horiz0 = np.sqrt(dx0**2 + dy0**2)
    psi_initial = np.arctan2(dy0, dx0)          # cap (lacet)
    gamma_initial = np.arctan2(dz0, horiz0)      # pente (tangage)

    psi_s = psi_initial
    gamma_s = gamma_initial
    v_s = 15.0
    a_prop = 12.0
    omega_psi_reel = 0.0
    omega_gamma_reel = 0.0

    # ---------
    # INITIALISATION EKF (vecteur d'état 12 composantes)
    # X = [xs, ys, zs, psi, gamma, vs, xt, yt, zt, vxt, vyt, vzt]
    # ---------
    X = np.array([0.0, 0.0, 0.0, psi_initial, gamma_initial, 15.0,
                  30.0, 120.0, 80.0, 1.5, -0.2, 0.3])

    P = np.eye(12) * 0.1
    bruits_diagonaux = [0.01, 0.01, 0.01,           # xs, ys, zs
                        std_gyro**2, std_gyro**2,   # psi, gamma
                        std_accel**2,                # vs
                        0.05, 0.05, 0.05,             # xt, yt, zt
                        0.5, 0.5, 0.5]                # vxt, vyt, vzt
    Q = np.diag(bruits_diagonaux)
    R = np.diag([std_ir**2, std_ir**2])  # mesure = [azimut_relatif, élévation_relative]
    I = np.eye(12)

    last_los_az = None
    last_los_el = None

    # Historiques
    history_true_target = []
    history_true_suiveur = []
    history_ekf_target = []
    history_signal_status = []
    history_time = []
    history_v_s = []
    history_heading = []
    history_pitch = []
    history_g_force = []

    sens_zigzag = 2

    # ---------
    # BOUCLE DE SIMULATION
    # ---------
    for k in range(temps):
        t_courant = k * dt
        history_time.append(t_courant)

        # 1. BROUILLAGE / PERTE DE SIGNAL
        signal_perdu = perte_signal_active and (50 <= k <= 100)
        history_signal_status.append(signal_perdu)

        # 2. MANŒUVRE D'ESQUIVE BRUTALE (horizontale + rupture verticale)
        if manoeuvre_esquive_active and k == 120:
            vx_t_old, vz_t_old = vx_t, vz_t
            vx_t = -vy_t * 1.5
            vy_t = vx_t_old * 1.5
            vz_t = -vz_t_old * 1.2 - 0.4  # rupture en piqué/ressource

        # 3. ZIGZAG PÉRIODIQUE (dans le plan horizontal)
        if manoeuvre_zigzag_active and k > 20 and (k % 60 == 0):
            angle_virage = np.radians(35.0) * sens_zigzag
            c, s = np.cos(angle_virage), np.sin(angle_virage)
            vx_t_new = c * vx_t - s * vy_t
            vy_t = s * vx_t + c * vy_t
            vx_t = vx_t_new
            sens_zigzag *= -1

        # 4. LOI DE GUIDAGE (PRONAV 3D DÉCOUPLÉE AZIMUT / ÉLÉVATION)
        xs_e, ys_e, zs_e, psi_e, gamma_e, vs_e, xt_e, yt_e, zt_e, _, _, _ = X

        dx_e = xt_e - xs_e
        dy_e = yt_e - ys_e
        dz_e = zt_e - zs_e
        horiz_e = np.sqrt(dx_e**2 + dy_e**2)

        los_az = np.arctan2(dy_e, dx_e)
        los_el = np.arctan2(dz_e, horiz_e)

        if last_los_az is None:
            omega_psi_commandee = 0.0
            omega_gamma_commandee = 0.0
        else:
            d_az = (los_az - last_los_az + np.pi) % (2 * np.pi) - np.pi
            d_el = (los_el - last_los_el + np.pi) % (2 * np.pi) - np.pi
            los_az_dot = d_az / dt
            los_el_dot = d_el / dt
            omega_psi_commandee = N_gain * los_az_dot
            omega_gamma_commandee = N_gain * los_el_dot

        last_los_az, last_los_el = los_az, los_el

        # --- CONTRAINTES DE VITESSE ANGULAIRE (slew rate), par axe ---
        diff_omega_max = MAX_SLEW_RATE_OMEGA * dt

        diff_psi = omega_psi_commandee - omega_psi_reel
        omega_psi_reel += np.clip(diff_psi, -diff_omega_max, diff_omega_max)

        diff_gamma = omega_gamma_commandee - omega_gamma_reel
        omega_gamma_reel += np.clip(diff_gamma, -diff_omega_max, diff_omega_max)

        # --- CONTRAINTE DE FACTEUR DE CHARGE (G-limit), combinée ---
        a_lat_psi = v_s * np.cos(gamma_s) * omega_psi_reel
        a_lat_gamma = v_s * omega_gamma_reel
        a_lat_norm = np.sqrt(a_lat_psi**2 + a_lat_gamma**2)
        if a_lat_norm > MAX_LATERAL_ACCEL and a_lat_norm > 1e-9:
            facteur = MAX_LATERAL_ACCEL / a_lat_norm
            omega_psi_reel *= facteur
            omega_gamma_reel *= facteur
            a_lat_psi *= facteur
            a_lat_gamma *= facteur

        g_force_actuelle = np.sqrt(a_lat_psi**2 + a_lat_gamma**2) / ACCEL_GRAVITE
        history_g_force.append(abs(g_force_actuelle))

        # --- TRAÎNÉE AÉRODYNAMIQUE ET GRAVITÉ (le long de l'axe de vol) ---
        F_drag = 0.5 * Rho * S_ref * Cx * (v_s**2)
        a_drag = F_drag / Masse
        a_gravite = -ACCEL_GRAVITE * np.sin(gamma_s)
        a_net_axiale = a_prop - a_drag + a_gravite

        u_true = np.array([a_net_axiale, omega_psi_reel, omega_gamma_reel])

        # 5. ÉVOLUTION DU VRAI SYSTÈME PHYSIQUE (3D)
        x_t += vx_t * dt
        y_t += vy_t * dt
        z_t += vz_t * dt

        v_s = max(v_s + u_true[0] * dt, 0.5)
        psi_s = psi_s + u_true[1] * dt
        gamma_s = gamma_s + u_true[2] * dt

        vx_s = v_s * np.cos(gamma_s) * np.cos(psi_s)
        vy_s = v_s * np.cos(gamma_s) * np.sin(psi_s)
        vz_s = v_s * np.sin(gamma_s)

        x_s += vx_s * dt
        y_s += vy_s * dt
        z_s += vz_s * dt

        dist_phys = np.sqrt((x_t - x_s)**2 + (y_t - y_s)**2 + (z_t - z_s)**2)

        # 6. CAP BOUSSOLE ET ANGLE DE PENTE (pour l'affichage)
        heading_boussole = (90.0 - np.degrees(psi_s)) % 360.0
        history_heading.append(heading_boussole)
        history_pitch.append(np.degrees(gamma_s))

        # 7. CAPTEURS BRUITÉS
        u_measured = u_true + np.array([
            np.random.normal(0, std_accel),
            np.random.normal(0, std_gyro),
            np.random.normal(0, std_gyro)
        ])

        dx_true = x_t - x_s
        dy_true = y_t - y_s
        dz_true = z_t - z_s
        horiz_true = np.sqrt(dx_true**2 + dy_true**2)

        z_az = (np.arctan2(dy_true, dx_true) - psi_s) + np.random.normal(0, std_ir)
        z_el = (np.arctan2(dz_true, horiz_true) - gamma_s) + np.random.normal(0, std_ir)

        # 8. FILTRE DE KALMAN ÉTENDU (EKF) — PRÉDICTION
        xs, ys, zs, psi, gamma, vs, xt, yt, zt, vxt, vyt, vzt = X
        ax, om_psi, om_gamma = u_measured

        cg, sg = np.cos(gamma), np.sin(gamma)
        cp, sp = np.cos(psi), np.sin(psi)

        X_pred = np.array([
            xs + vs * cg * cp * dt,
            ys + vs * cg * sp * dt,
            zs + vs * sg * dt,
            psi + om_psi * dt,
            gamma + om_gamma * dt,
            vs + ax * dt,
            xt + vxt * dt,
            yt + vyt * dt,
            zt + vzt * dt,
            vxt,
            vyt,
            vzt
        ])

        G = np.eye(12)
        # dérivées de xs
        G[0, 3] = -vs * cg * sp * dt          # d(xs)/d(psi)
        G[0, 4] = -vs * sg * cp * dt          # d(xs)/d(gamma)
        G[0, 5] = cg * cp * dt                # d(xs)/d(vs)
        # dérivées de ys
        G[1, 3] = vs * cg * cp * dt
        G[1, 4] = -vs * sg * sp * dt
        G[1, 5] = cg * sp * dt
        # dérivées de zs
        G[2, 4] = vs * cg * dt
        G[2, 5] = sg * dt
        # xt, yt, zt dépendent de vxt, vyt, vzt (déjà à 1.0 sur diag, ajout dt)
        G[6, 9] = dt
        G[7, 10] = dt
        G[8, 11] = dt

        P = G @ P @ G.T + Q
        X = X_pred.copy()

        if not signal_perdu:
            xs, ys, zs, psi, gamma, vs, xt, yt, zt, vxt, vyt, vzt = X
            dx = xt - xs
            dy = yt - ys
            dz = zt - zs
            r2 = dx**2 + dy**2
            h = np.sqrt(r2) if r2 > 1e-6 else 1e-6
            r3sq = r2 + dz**2
            r3sq = r3sq if r3sq > 1e-6 else 1e-6

            if r2 > 0.01:
                az_pred = np.arctan2(dy, dx) - psi
                el_pred = np.arctan2(dz, h) - gamma

                y_az = (z_az - az_pred + np.pi) % (2 * np.pi) - np.pi
                y_el = (z_el - el_pred + np.pi) % (2 * np.pi) - np.pi
                y_val = np.array([y_az, y_el])

                H = np.zeros((2, 12))
                # ligne azimut
                H[0, 0] = dy / r2
                H[0, 1] = -dx / r2
                H[0, 3] = -1.0
                H[0, 6] = -dy / r2
                H[0, 7] = dx / r2
                # ligne élévation
                H[1, 0] = dz * dx / (r3sq * h)
                H[1, 1] = dz * dy / (r3sq * h)
                H[1, 2] = -h / r3sq
                H[1, 4] = -1.0
                H[1, 6] = -dz * dx / (r3sq * h)
                H[1, 7] = -dz * dy / (r3sq * h)
                H[1, 8] = h / r3sq

                S = H @ P @ H.T + R
                K = P @ H.T @ np.linalg.inv(S)

                X = X + K @ y_val
                P = (I - K @ H) @ P

        # ENREGISTREMENT DES HISTORIQUES
        history_true_target.append([x_t, y_t, z_t])
        history_true_suiveur.append([x_s, y_s, z_s])
        history_ekf_target.append([X[6], X[7], X[8]])
        history_v_s.append(v_s)

        # 9. CONDITION D'INTERCEPTION PHYSIQUE RÉELLE
        if k > 20 and dist_phys < 2.5:
            print(f"Cible interceptée avec succès à t = {t_courant:.2f}s !")
            break

    # Conversions finales
    history_true_target = np.array(history_true_target)
    history_true_suiveur = np.array(history_true_suiveur)
    history_ekf_target = np.array(history_ekf_target)
    history_signal_status = np.array(history_signal_status)
    history_time = np.array(history_time)
    history_v_s = np.array(history_v_s)
    history_heading = np.array(history_heading)
    history_pitch = np.array(history_pitch)
    history_g_force = np.array(history_g_force)

    # ---------
    # RENDU GRAPHIQUE
    # ---------
    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(3, 4, height_ratios=[2.2, 1, 1])

    idx_perte = np.where(history_signal_status == True)[0]  # noqa: E712

    def surligner_brouillage(axis):
        if len(idx_perte) > 0:
            axis.axvspan(history_time[idx_perte[0]], history_time[idx_perte[-1]],
                         color='purple', alpha=0.12, label="Brouillage Actif")

    # --- 1. TRAJECTOIRE SPATIALE 3D ---
    ax1 = fig.add_subplot(gs[0, :], projection='3d')
    ax1.plot(history_true_suiveur[:, 0], history_true_suiveur[:, 1], history_true_suiveur[:, 2],
              'b-', label="Projectile (ProNav)", linewidth=2.5)
    ax1.plot(history_true_target[:, 0], history_true_target[:, 1], history_true_target[:, 2],
              'r-', label="Cible Réelle (Chaleur IR)", linewidth=2.5)
    ax1.plot(history_ekf_target[:, 0], history_ekf_target[:, 1], history_ekf_target[:, 2],
              color='orange', linestyle="--", label="Cible Estimée (EKF)", alpha=0.85)

    if len(idx_perte) > 0:
        ax1.plot(history_true_suiveur[idx_perte, 0], history_true_suiveur[idx_perte, 1],
                  history_true_suiveur[idx_perte, 2], color='purple', linewidth=5,
                  label="Brouillage Capteur")

    ax1.scatter(*history_true_suiveur[-1], color='green', s=150, label="Impact")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Altitude Z (m)")
    ax1.set_title("Guidage Spatial 3D")
    ax1.legend(loc="upper left", fontsize=8)

    # --- 2. VITESSE ---
    ax2 = fig.add_subplot(gs[1, 0:2])
    ax2.plot(history_time, history_v_s, 'darkblue', linewidth=2.5, label="Vitesse Réelle (m/s)")
    surligner_brouillage(ax2)
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Vitesse (m/s)")
    ax2.set_title("Profil de Vitesse")
    ax2.legend(loc="upper left", fontsize=8)
    ax2.grid(True)

    # --- 3. CAP BOUSSOLE (lacet) ---
    ax3 = fig.add_subplot(gs[1, 2:4])
    ax3.plot(history_time, history_heading, 'darkgreen', linewidth=2.5, label="Cap Boussole (°)")
    surligner_brouillage(ax3)
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("Orientation Boussole")
    ax3.set_title("Évolution du Cap (Lacet)")
    ticks_angles = np.arange(0, 361, 90)
    ax3.set_yticks(ticks_angles)
    ax3.set_yticklabels(["000° (N)", "090° (E)", "180° (S)", "270° (O)", "360° (N)"])
    ax3.set_ylim(-10, 370)
    ax3.legend(loc="upper left", fontsize=8)
    ax3.grid(True)

    # --- 4. PENTE (tangage) ---
    ax4 = fig.add_subplot(gs[2, 0:2])
    ax4.plot(history_time, history_pitch, 'teal', linewidth=2.5, label="Angle de Pente (°)")
    surligner_brouillage(ax4)
    ax4.axhline(0, color='gray', linestyle=':', linewidth=1)
    ax4.set_xlabel("Temps (s)")
    ax4.set_ylabel("Pente (°)")
    ax4.set_title("Évolution de la Pente (Tangage)")
    ax4.legend(loc="upper left", fontsize=8)
    ax4.grid(True)

    # --- 5. FACTEUR DE CHARGE (G-force) ---
    ax5 = fig.add_subplot(gs[2, 2:4])
    ax5.plot(history_time, history_g_force, 'brown', linewidth=2.5, label="Accélération latérale (G)")
    ax5.axhline(G_LIMIT, color='red', linestyle='--', label="Limite Structurelle (20G)")
    surligner_brouillage(ax5)
    ax5.set_xlabel("Temps (s)")
    ax5.set_ylabel("Facteur de Charge (G)")
    ax5.set_title("Accélération Latérale Combinée (Lacet + Tangage)")
    ax5.legend(loc="upper left", fontsize=8)
    ax5.grid(True)

    plt.tight_layout()
    plt.savefig('simulation_3d.png', dpi=130)  # sauvegarde dans le dossier courant
    plt.show()

if __name__ == "__main__":
    lancer_simulation_3d()