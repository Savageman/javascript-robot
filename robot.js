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


// pcc = plus court chemin
// tableau dijkstra
var pcc = new Array();
var liste_stations = new Array();

function init_custom() {
	for (j = hauteur_terrain; j >= 1; j--) {
		pcc[j] = new Array();
		for (i = 1; i <= largeur_terrain; i++) {
			pcc[j][i] = 99;
		}
	}
	debug_pcc();
}
function debug_pcc() {
	for (j = hauteur_terrain; j >= 1; j--) {
		for (i = 1; i <= largeur_terrain; i++) {
			var str = '';
			if (j == robot_y && i == robot_x) {
				str += '<strong>';
			}
			if (terrain_explore[j][i] == 9) {
				str += '<img src="img/inconnu.gif" />';
			} else if (terrain_explore[j][i] == 1) {
				str += '<img src="img/obstacle.gif" />';
			} else {
				str += pcc[j][i] == 99 ? '.' : pcc[j][i];
			}
			if (j == robot_y && i == robot_x) {
				str += '</strong>';
			}
			document.getElementById('pcc_' + j + '_' + i).innerHTML = str;
		}
	}
}
init_custom();

function dijkstra_maj_cell_recursive(y, x, dist, toLeft, toRight, toTop, toBottom, onlyUnknown)
{
	// OutOfBounds ou déjà plus court
	if (y <= 0 || y > hauteur_terrain || x <= 0 || x > largeur_terrain) {
		return;
	}
	if (onlyUnknown && dist > 0 && pcc[y][x] < 99) {
		return;
	}
	var cell = terrain_explore[y][x];
	// 1 = obstacle, 9 == inexploré
	if (cell == 1 || cell == 9) {
		return;
	}

	count_recursive++;

	if (dist < pcc[y][x]) { 
		pcc[y][x] = dist;
	} else {
		dist = pcc[y][x];
	}

	// Récursion
	if (toTop && typeof pcc[y + 1] != 'undefined') {
		dijkstra_maj_cell_recursive(y + 1, x, dist + 1, toLeft, toRight, toTop, false, onlyUnknown);
	}
	if (toBottom && typeof pcc[y - 1] != 'undefined') {
		dijkstra_maj_cell_recursive(y - 1, x, dist + 1, toLeft, toRight, false, toBottom, onlyUnknown);
	}
	if (toLeft && typeof pcc[y][x - 1] != 'undefined') {
		dijkstra_maj_cell_recursive(y, x - 1, dist + 1, toLeft, false, toTop, toBottom, onlyUnknown);
	}
	if (toRight && typeof pcc[y][x + 1] != 'undefined') {
		dijkstra_maj_cell_recursive(y, x + 1, dist + 1, false, toRight, toTop, toBottom, onlyUnknown);
	}
}

function dijkstra_maj_unknown_cell(y, x) {
	if (pcc[y][x] < 99) {
		//console.log('skip', y, x, pcc[y][x]);
		//return false;
	}
	// Recherche du minimal adjacent
	var min = [pcc[y][x]];
	if (y > 1) {
		min.push(pcc[y - 1][x]);
	}
	if (y < hauteur_terrain) {
		min.push(pcc[y + 1][x]);
	}
	if (x > 1) {
		min.push(pcc[y][x - 1]);
	}
	if (x < largeur_terrain) {
		min.push(pcc[y][x + 1]);
	}
	min = Math.min.apply(null, min);
	console.log(y, x, "=", min);
	dijkstra_maj_cell_recursive(y, x, min + 1, true, true, true, true);
	return true;
}

function ajoute_station(j, i) {
	if (typeof liste_stations[j + '_' + i] == 'undefined') {
		liste_stations[j + '_' + i] = [j, i];
		dijkstra_maj_cell_recursive(j, i, 0, true, true, true, true);
	}
}

function decider_direction(CH, CB, CG, CD, but_dir_hb, but_dir_gd, but_dist, reserve_carburant) {

	count_recursive = 0;
	for (station in liste_stations) {
		dijkstra_maj_cell_recursive(liste_stations[station][0], liste_stations[station][1], 0, true, true, true, true, true);
	}
	
	console.log('maj station', count_recursive);
	// On voit une station
	if (CH.voit == 3) {
		ajoute_station(robot_y + CH.dist, robot_x);
	}
	if (CB.voit == 3) {
		ajoute_station(robot_y - CB.dist, robot_x);
	}
	if (CD.voit == 3) {
		ajoute_station(robot_y, robot_x + CD.dist);
	}
	if (CG.voit == 3) {
		ajoute_station(robot_y, robot_x - CG.dist);
	}
	console.log('add station', count_recursive);

	// Mise à jour case courante
	dijkstra_maj_unknown_cell(robot_y, robot_x);
	console.log('robot', count_recursive);

	debug_pcc();

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
		'robot_x = '+robot_x+', robot_y = ' + robot_y + '<br />'
		 + 'CH.voit = '+CH.voit+', CH.dist = '+CH.dist + '<br />'
		 +' CB.voit = '+CB.voit+', CB.dist = '+CB.dist + '<br />'
		 +' CG.voit = '+CG.voit+', CG.dist = '+CG.dist + '<br />'
		 +' CD.voit = '+CD.voit+', CD.dist = '+CD.dist + '<br />'
		 + ' but_dir_hb = ' + but_dir_hb + ', but_dir_gd = ' + but_dir_gd + '<br />'
		 + ' but_dist = ' + but_dist + ', reserve_carburant = ' + reserve_carburant;

	var ret = reserve_carburant % 2 == 0 ? "D" : "H";

	return ret;

}