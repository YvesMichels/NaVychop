#ifndef CONFIG_H
#define CONFIG_H

#define size_msg 5 // taille des messages vers le micro-controleur des moteurs de l'avion

#define nb_compress 50 // nombre de données par paquet (50@20Hz => toutes les 2.5 secondes)
#define nb_paquet 100 // taille du buffer de données "init" à envoyer au sol (100*2.5 secondes => 250 secondes)
#define size_buffer_delta 4950 // taille du buffer de données "delta" à envoyer au sol size_buffer_delta = nb_paquet*(nb_compress-1)
#define nb_it_sparce 20 // nombre d'it entre deux envois "éparses" (20@20Hz => 1 par seconde)
#define size_buffer_sparce 250 // envoi plus lent (250*1 secondes => 250 secondes)

#define max_cpt_lora 3000 // temps max autorisé sans communication (ms)
#define d_t 40 // interval de temps cible d'une boucle (ms)
#define binpt_yaw 8 // mise à l'échelle yaw (puissance de 2 car quantification en virgule fixe)
#define binpt_pitch 4 // mise à l'échelle pitch (puissance de 2 car quantification en virgule fixe)
#define binpt_roll 4 // mise à l'échelle roll (puissance de 2 car quantification en virgule fixe)

/* configuration défaut PID */
#define def_kp_yaw 20        // en cmd_moteur / projection_axe ([-32;31]/[-1;1])
#define def_ki_yaw 0.005    // en cmd_moteur / (projection_axe*ms) ([-32;31]/[-def_max_eta_yaw;def_max_eta_yaw])
#define def_kd_yaw 10        // en cmd_moteur / (projection_axe/ms) (dt*[-32;31]/[-1;1])
#define def_kp_pitch 30      // en cmd_moteur / projection_axe ([-32;31]/[-1;1])
#define def_ki_pitch 0.01   // en cmd_moteur / (projection_axe*ms) ([-32;31]/[-def_max_eta_pitch;def_max_eta_pitch])
#define def_kd_pitch 30     // en cmd_moteur / (projection_axe/ms) (dt*[-32;31]/[-1;1])
#define def_kp_roll 30       // en cmd_moteur / projection_axe ([-32;31]/[-1;1])
#define def_ki_roll 0.04    // en cmd_moteur / (projection_axe*ms) ([-32;31]/[-def_max_eta_roll;def_max_eta_roll])
#define def_kd_roll 20      // en cmd_moteur / (projection_axe/ms) (dt*[-32;31]/[-1;1])
#define def_max_eta_yaw 300     // 0.3 d'écart pendant une seconde
#define def_max_eta_pitch 300   // 0.3 d'écart pendant une seconde
#define def_max_eta_roll 300    // 0.3 d'écart pendant une seconde

#endif