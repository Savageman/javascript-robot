/*
 * fonction à compléter, permettant de décider de la direction vers laquelle diriger votre robot.
 *
 * Il cherchera alors à se déplacer d'une case dans la direction décidée.
 * - Si cette case est vide (ou est la case "but"), votre robot s'y rendra.
 * - Si cette case est une ressource, il ne bougera pas mais remplira son réservoir de carburant.
 * - Si cette case est un obstacle ou qu'il tente de sortir des limites du terrain, votre robot ne bougera pas mais
 *   consommera 1 mL de carburant (comme s'il s'était déplacé d'une case).
 *
 *
 * PARAMETRES :
 *
 * CH : Capteur Haut (ou "capteur avant")
 *     CH.voit = ce que "voit" le Capteur Haut (> 0)
 *     CH.dist = nb de cases entre ce qui est "vu" par le Capteur Haut, et le robot (1 = adjacent)
 * CB : Capteur Bas (ou "capteur arrière")
 *     CB.voit = ce que "voit" le Capteur Bas (> 0)
 * etc...
 *
 * but_dir_hb = 1, 0 ou -1 (dans quelle direction est le but ? 1 = plus haut,     -1 = plus bas,      0 = ok)
 * but_dir_gd = 1, 0 ou -1 (dans quelle direction est le but ? 1 = plus à droite, -1 = plus à gauche, 0 = ok)
 * but_dist : distance (arrondie au dixième près) entre le robot et le but, en nombre de cases (1.0 = adjacent)
 *
 * reserve_carburant : carburant restant dans le réservoir du robot, en mL. S'il atteint zéro vous avez perdu...
 *
 * (remarque : il est interdit d'utiliser la variable "terrain"...)
 *
 * VARIABLES ACCESSIBLES NON DONNEES EN PARAMETRE
 *
 * but_x : abscisse du but
 * but_y : ordonnée du but
 *
 * terrain_explore : un tableau [1..hauteur_terrain][1..largeur_terrain] contenant le terrain explore par le robot
 *     une case peut contenir les entiers suivants :
 *        0 = terrain vide
 *        1 = obstacle
 *        2 = but
 *        3 = ressource
 *        4 = position du robot
 *        9 = case inconnue du robot (jamais vue par l'un de ses capteurs)
 *
 * RETOURNER UNE DIRECTION : "H", "B", "G" ou "D".
 * un déplacement dans un obstacle (bords du terrain, ou cases avec le nombre 1) aura pour résultat :
 * - le robot ne bougera pas
 * - le robot aura consommé 1 mL de carburant
 */


distance_carburant = terrain_explore;

function decider_direction(CH, CB, CG, CD, but_dir_hb, but_dir_gd, but_dist, reserve_carburant) {
    // But visible et accessible => go
    if (CH.voit==2 && CH.dist <= reserve_carburant) { return 'H'; }
    if (CB.voit==2 && CB.dist <= reserve_carburant) { return 'B'; }
    if (CG.voit==2 && CG.dist <= reserve_carburant) { return 'G'; }
    if (CD.voit==2 && CD.dist <= reserve_carburant) { return 'D'; }

    // Une seule direction possible...
    if (CD.dist + CB.dist + CG.dist == 3) { return 'H'; }
    if (CH.dist + CG.dist + CD.dist == 3) { return 'B'; }
    if (CH.dist + CD.dist + CB.dist == 3) { return 'G'; }
    if (CH.dist + CG.dist + CB.dist == 3) { return 'D'; }

    document.getElementById('debug').innerHTML =
        'CH.voit = '+CH.voit+', CH.dist = '+CH.dist + '<br />'
         +' CB.voit = '+CB.voit+', CB.dist = '+CB.dist + '<br />'
         +' CG.voit = '+CG.voit+', CG.dist = '+CG.dist + '<br />'
         +' CD.voit = '+CD.voit+', CD.dist = '+CD.dist + '<br />'
         + ' but_dir_hb = ' + but_dir_hb + ', but_dir_gd = ' + but_dir_gd + '<br />'
         + ' but_dist = ' + but_dist + ', reserve_carburant = ' + reserve_carburant;

    return reserve_carburant % 2 == 0 ? "D" : "H";

}




