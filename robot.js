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
var pcc_mark = new Array();
var astar_map = new Array();
var astar_weight = new Array();
var liste_stations = new Array();
var cell_queue = new Queue();
var cell_exists = new Queue();

function init_custom() {
	for (j = hauteur_terrain; j >= 1; j--) {
		pcc[j] = new Array();
		astar_weight[j - 1] = new Array();
		for (i = 1; i <= largeur_terrain; i++) {
			pcc[j][i] = 99;
			astar_weight[j - 1][i - 1] = 1;
		}
	}
	debug_pcc();
}

function init_mark() {
	for(j = 1; j <= hauteur_terrain; j++) {
		pcc_mark[j] = new Array();
		for (i = 1; i <= largeur_terrain; i++) {
			pcc_mark[j][i] = false;
		}
	}
}

function reset_mark() {
	for(j = 1; j <= hauteur_terrain; j++) {
		for (i = 1; i <= largeur_terrain; i++) {
			pcc_mark[j][i] = false;
		}
	}
}

/* Debug procedures */
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

function debug_astar(chemin) {
	for (j = 0; j < hauteur_terrain; j++) {
		for (i = 0; i < largeur_terrain; i++) {
			var str = '';
			if (astar_map[j][i] == 0) {
				str += '<img src="img/obstacle.gif" />';
			}
			document.getElementById('astar_' + j + '_' + i).innerHTML = str;
		}
	}
	var distance = 0;
	for (cell in chemin) {
		document.getElementById('astar_' + chemin[cell].x + '_' + chemin[cell].y).innerHTML = distance;
		distance++;
	}
}
/* End of debug */

init_custom();
init_mark();

function dijkstra_bfs_maj_ukn(y, x) {
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
	dijkstra_bfs_maj(y, x, min + 1, true);
	return true;
}



/* Ajoute une case à la file d'attente */
function addToQueue(y, x, dist) {
    /* If node has not already been visited */
    if(pcc_mark[y][x] == false ) {
        if(terrain_explore[y][x] != 1 && terrain_explore[y][x] != 9) {
            /* If it is not already on the list */
            var cell = cell_exists.indexOf(y + '_' + x);

            if(cell == -1) {
                cell_exists.enqueue(y + '_' + x);
                cell_queue.enqueue([y, x, dist]);
            } else {
                var  val = cell_queue.get(cell);
                if(val[2] > dist) {
                    val[2] = dist;
                    cell_queue.put(cell, val);
                }
            }
        }
    }
}

/* BFS Dijkstra */
function dijkstra_bfs_maj(y, x, dist, onlyUnknown) {
    /* Reset the variables */
    cell_queue.reset();
    cell_exists.reset();
    reset_mark();

    /* Ajoute la première case à la file */
    addToQueue(y, x, dist);

    while(!cell_queue.isEmpty()) {
        /* The working node */
        var step = cell_queue.dequeue();
        cell_exists.dequeue();

        /* If no value in cell */
        if((pcc[step[0]][step[1]] < 99 /*&& !onlyUnknown*/) 
            || (pcc[step[0]][step[1]] == 99)) {
            /* If dist is lower */
            if(pcc[step[0]][step[1]] > step[2]) {
                pcc[step[0]][step[1]] = step[2];
            }
        }

        pcc_mark[step[0]][step[1]] = true;

        /* Adding all the neighbour to the queue */
        var x = step[1];
        var y = step[0];
        var d = step[2] + 1;

        if(typeof pcc[y + 1] != 'undefined') {
            addToQueue(y + 1, x, d);
        }
        if(typeof pcc[y - 1] != 'undefined') {
            addToQueue(y - 1, x, d);
        }
        if(typeof pcc[y][x + 1] != 'undefined') {
            addToQueue(y, x + 1, d);
        }
        if(typeof pcc[y][x - 1] != 'undefined') {
            addToQueue(y, x - 1, d);
        }
    }
}

function ajoute_station(j, i) {
	if (typeof liste_stations[j + '_' + i] == 'undefined') {
		liste_stations[j + '_' + i] = [j, i];
		//dijkstra_maj_cell_recursive(j, i, 0, true, true, true, true);
		dijkstra_bfs_maj(j, i, 0);
	}
	return false;
}

function chemin_astar()
{
	// 1. On construit la carte pour l'algorithme
	for (var j = 0; j < hauteur_terrain; j++) {
		astar_map[j] = new Array();
		for (var i = 0; i < largeur_terrain; i++) {
			var cell = terrain_explore[j+1][i+1];
			/* 0 = terrain vide
			 * 1 = obstacle
			 * 2 = but
			 * 3 = ressource
			 * 4 = position du robot */
			astar_map[j][i] = cell == 1  || cell == 3 ? 0 : astar_weight[j][i];
		}
	}
    var graph = new Graph(astar_map);
    var start = graph.grid[robot_y - 1][robot_x - 1];
    var end = graph.grid[but_y - 1][but_x - 1];
    var result = astar.search(graph, start, end);
	return result;
}

function decider_direction(CH, CB, CG, CD, but_dir_hb, but_dir_gd, but_dist, reserve_carburant) {

	count_recursive = 0;
	for (station in liste_stations) {
		dijkstra_bfs_maj(liste_stations[station][0], liste_stations[station][1], 0, true);
	}
	
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

	// Mise à jour case courante
	//dijkstra_maj_unknown_cell(robot_y, robot_x);
    dijkstra_bfs_maj_ukn(robot_y, robot_x);
    

	debug_pcc();

	// But visible et accessible => go
	if (CH.voit==2 && CH.dist <= reserve_carburant) { return 'H'; }
	if (CB.voit==2 && CB.dist <= reserve_carburant) { return 'B'; }
	if (CG.voit==2 && CG.dist <= reserve_carburant) { return 'G'; }
	if (CD.voit==2 && CD.dist <= reserve_carburant) { return 'D'; }
	
	
	var pcc_robot = pcc[robot_y][robot_x];
	if (reserve_carburant < 30 && pcc_robot >= reserve_carburant - 1) {
		if (typeof pcc[robot_y + 1] != 'undefined' && pcc[robot_y + 1][robot_x] < pcc_robot) {
			return "H";
		}
		if (typeof pcc[robot_y - 1] != 'undefined' && pcc[robot_y - 1][robot_x] < pcc_robot) {
			return "B";
		}
		if (typeof pcc[robot_y][robot_x - 1] != 'undefined' && pcc[robot_y][robot_x - 1] < pcc_robot) {
			return "G";
		}
		if (typeof pcc[robot_y][robot_x + 1] != 'undefined' && pcc[robot_y][robot_x + 1] < pcc_robot) {
			return "D";
		}
	}

	// Une seule direction possible...
	if (CD.dist + CB.dist + CG.dist == 3) { return 'H'; }
	if (CH.dist + CG.dist + CD.dist == 3) { return 'B'; }
	if (CH.dist + CD.dist + CB.dist == 3) { return 'G'; }
	if (CH.dist + CG.dist + CB.dist == 3) { return 'D'; }
	
	var chemin = chemin_astar();
	var prochain_x = chemin[0].y + 1;
	var prochain_y = chemin[0].x + 1;
	
	debug_astar(chemin);
		
	if (prochain_x > robot_x) {
		astar_weight[robot_y - 1][robot_x]++;
		return "D";
	}
	if (prochain_x < robot_x) {
		astar_weight[robot_y - 1][robot_x - 2]++;
		return "G";
	}
	if (prochain_y > robot_y) {
		astar_weight[robot_y][robot_x - 1]++;
		return "H";
	}
	if (prochain_y < robot_y) {
		astar_weight[robot_y - 2][robot_x - 1]++;
		return "B";
	}
	
	document.getElementById('debug').innerHTML =
		'robot_x = '+robot_x+', robot_y = ' + robot_y + '<br />'
		 + 'CH.voit = '+CH.voit+', CH.dist = '+CH.dist + '<br />'
		 +' CB.voit = '+CB.voit+', CB.dist = '+CB.dist + '<br />'
		 +' CG.voit = '+CG.voit+', CG.dist = '+CG.dist + '<br />'
		 +' CD.voit = '+CD.voit+', CD.dist = '+CD.dist + '<br />'
		 + ' but_dir_hb = ' + but_dir_hb + ', but_dir_gd = ' + but_dir_gd + '<br />'
		 + ' but_dist = ' + but_dist + ', reserve_carburant = ' + reserve_carburant;

	return reserve_carburant % 2 == 0 ? "D" : "H";
}
