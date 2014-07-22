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
var pcc = [];
var pcc_mark = [];
var pcc_goal = [];
var astar_map = [];
var astar_fake_wall_list = [];
var liste_stations = [];
var cell_queue = new Queue();
var cell_exists = new Queue();

function init_custom() {
	for (var j = hauteur_terrain; j >= 1; j--) {
		pcc[j] = [];
        pcc_goal[j] = [];
        pcc_mark[j] = [];
		for (var i = 1; i <= largeur_terrain; i++) {
			pcc[j][i] = 99;
            pcc_goal[j][i] = 99;
            pcc_mark[j][i] = false;
		}
	}
	debug_pcc();
    debug_pcc_goal();
}

function bfs_reset_mark() {
	for(var j = 1; j <= hauteur_terrain; j++) {
		for (var i = 1; i <= largeur_terrain; i++) {
			pcc_mark[j][i] = false;
        }
    }
}

/* Debug procedures */
function debug_pcc() {
	for (var j = hauteur_terrain; j >= 1; j--) {
		for (var i = 1; i <= largeur_terrain; i++) {
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

function debug_pcc_goal() {
	for (var j = hauteur_terrain; j >= 1; j--) {
		for (var i = 1; i <= largeur_terrain; i++) {
			var str = '';
			if (j == robot_y && i == robot_x) {
				str += '<strong>';
			}
			if (terrain_explore[j][i] == 9) {
				str += '<img src="img/inconnu.gif" />';
			} else if (terrain_explore[j][i] == 1) {
				str += '<img src="img/obstacle.gif" />';
			} else {
				str += pcc_goal[j][i] == 99 ? '.' : pcc_goal[j][i];
			}
			if (j == robot_y && i == robot_x) {
				str += '</strong>';
			}
			document.getElementById('pcc_goal_' + j + '_' + i).innerHTML = str;
		}
	}
}

function debug_astar(chemin) {
	for (var j = 0; j < hauteur_terrain; j++) {
		for (var i = 0; i < largeur_terrain; i++) {
			var str = '';
			if (astar_map[j][i] == 0) {
                if (astar_fake_wall_list[(j+1)+'_'+(i+1)]) {
                    str += '<img src="img/obstacle_grey.gif" />';
                } else {
				    str += '<img src="img/obstacle.gif" />';
                }
			}
			document.getElementById('astar_' + j + '_' + i).innerHTML = str;
		}
	}
    var distance = 0;
   	for (var cell in chemin) {
   		document.getElementById('astar_' + chemin[cell].x + '_' + chemin[cell].y).innerHTML = distance;
   		distance++;
   	}
}
/* End of debug */

init_custom();

function dijkstra_bfs_maj_ukn(tab, y, x) {
	// Recherche du minimal adjacent
	var min = [tab[y][x]];
	if (y > 1) {
		min.push(tab[y - 1][x]);
	}
	if (y < hauteur_terrain) {
		min.push(tab[y + 1][x]);
	}
	if (x > 1) {
		min.push(tab[y][x - 1]);
	}
	if (x < largeur_terrain) {
		min.push(tab[y][x + 1]);
	}
	min = Math.min.apply(null, min);
	dijkstra_bfs_maj(tab, y, x, min + 1, true);
	return true;
}



/* Ajoute une case à la file d'attente */
function bfs_addToQueue(y, x, dist) {
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
function dijkstra_bfs_maj(tab, y, x, dist, onlyUnknown) {
    /* Reset the variables */
    cell_queue.reset();
    cell_exists.reset();
    bfs_reset_mark();

    /* Ajoute la première case à la file */
    bfs_addToQueue(y, x, dist);

    while(!cell_queue.isEmpty()) {
        /* The working node */
        var step = cell_queue.dequeue();
        cell_exists.dequeue();

        /* If no value in cell */
        if((tab[step[0]][step[1]] < 99 /*&& !onlyUnknown*/) 
            || (tab[step[0]][step[1]] == 99)) {
            /* If dist is lower */
            if(tab[step[0]][step[1]] > step[2]) {
                tab[step[0]][step[1]] = step[2];
            }
        }

        pcc_mark[step[0]][step[1]] = true;

        /* Adding all the neighbour to the queue */
        x = step[1];
        y = step[0];
        var d = step[2] + 1;

        if(typeof tab[y + 1] != 'undefined') {
            bfs_addToQueue(y + 1, x, d);
        }
        if(typeof tab[y - 1] != 'undefined') {
            bfs_addToQueue(y - 1, x, d);
        }
        if(typeof tab[y][x + 1] != 'undefined') {
            bfs_addToQueue(y, x + 1, d);
        }
        if(typeof tab[y][x - 1] != 'undefined') {
            bfs_addToQueue(y, x - 1, d);
        }
    }
}

function ajoute_station(j, i) {
    if (typeof liste_stations[j + '_' + i] == 'undefined') {
        liste_stations[j + '_' + i] = [j, i];
        dijkstra_bfs_maj(pcc, j, i, 0);
    }
    return false;
}

function chemin_astar(to_y, to_x)
{
    to_y = to_y || but_y;
    to_x = to_x || but_x;
	// 1. On construit la carte pour l'algorithme
	for (var j = 0; j < hauteur_terrain; j++) {
		astar_map[j] = [];
		for (var i = 0; i < largeur_terrain; i++) {
			var cell = terrain_explore[j+1][i+1];
			/* 0 = terrain vide
			 * 1 = obstacle
			 * 2 = but
			 * 3 = ressource
			 * 4 = position du robot */
            if (astar_fake_wall_list[(j+1)+'_'+(i+1)]) {
                astar_map[j][i] = 0;
            } else if (cell == 1  || cell == 3) {
                // 1=wall, 3=fuel_station
                astar_map[j][i] = 0;
            } else {
                astar_map[j][i] = 1;
            }
		}
	}
    var graph = new Graph(astar_map);
    var start = graph.grid[robot_y - 1][robot_x - 1];
    var end = graph.grid[to_y - 1][to_x - 1];
    return astar.search(graph, start, end);
}

function find_nearest_explorable_cell()
{
    for (var distance = 1; distance < 15; distance++) {
        var min_j = Math.max(1, robot_y - distance);
        var max_j = Math.min(hauteur_terrain, robot_y + distance);
        var min_i = Math.max(1, robot_x - distance);
        var max_i = Math.min(largeur_terrain, robot_x + distance);
        for (var j = min_j; j <= max_j; j++) {
            for (var i = min_i; i <= max_i; i++) {
                if (terrain_explore[j][i] == 9 && pcc[j][i] == 99) {
                    var path = chemin_astar(j, i);
                    if (path.length > 0) {
                        return { x:path[0].y + 1, y: path[0].x + 1 };
                    }
                }
            }
        }
    }
}

function decider_direction(CH, CB, CG, CD, but_dir_hb, but_dir_gd, but_dist, reserve_carburant) {

	for (var station in liste_stations) {
		dijkstra_bfs_maj(pcc, liste_stations[station][0], liste_stations[station][1], 0, true);
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

    // On Maj le but
    dijkstra_bfs_maj(pcc_goal, but_y, but_x, 0);

    // Mise à jour case courante
    dijkstra_bfs_maj_ukn(pcc, robot_y, robot_x);

	debug_pcc();
    debug_pcc_goal();

    var debug_pcc_wall = {};
    for (var astar_wall_index in astar_fake_wall_list) {
        var astar_wall = astar_fake_wall_list[astar_wall_index];
        debug_pcc_wall[astar_wall_index] = pcc[astar_wall.y][astar_wall.x];
        if (pcc[astar_wall.y][astar_wall.x] < 15) {
            delete astar_fake_wall_list[astar_wall_index];
        }
    }

	// But visible et accessible => go
	if (CH.voit==2 && CH.dist <= reserve_carburant) { return 'H'; }
	if (CB.voit==2 && CB.dist <= reserve_carburant) { return 'B'; }
	if (CG.voit==2 && CG.dist <= reserve_carburant) { return 'G'; }
	if (CD.voit==2 && CD.dist <= reserve_carburant) { return 'D'; }
	
	var pcc_robot = pcc[robot_y][robot_x];
	if (reserve_carburant < 30 && pcc_robot >= reserve_carburant - 1) {
        var backtrack = false;

		if (typeof pcc[robot_y + 1] != 'undefined' && pcc[robot_y + 1][robot_x] < pcc_robot) {
			backtrack = "H";
		}
		if (typeof pcc[robot_y - 1] != 'undefined' && pcc[robot_y - 1][robot_x] < pcc_robot) {
            backtrack = "B";
		}
		if (typeof pcc[robot_y][robot_x - 1] != 'undefined' && pcc[robot_y][robot_x - 1] < pcc_robot) {
            backtrack = "G";
		}
		if (typeof pcc[robot_y][robot_x + 1] != 'undefined' && pcc[robot_y][robot_x + 1] < pcc_robot) {
            backtrack = "D";
		}
        // If we reached the maximum possible distance, then add a fake wall for the a-star algorithm and try a new path
        if (backtrack && pcc_robot == capacite_reservoir_carburant / 2) {
            astar_fake_wall_list[robot_y + '_' + robot_x] = {x:robot_x, y:robot_y};
        }
        return backtrack;
	}

	// Une seule direction possible...
	if (CD.dist + CB.dist + CG.dist == 3) { return 'H'; }
	if (CH.dist + CG.dist + CD.dist == 3) { return 'B'; }
	if (CH.dist + CD.dist + CB.dist == 3) { return 'G'; }
	if (CH.dist + CG.dist + CB.dist == 3) { return 'D'; }
	
	var chemin = chemin_astar(but_y, but_x);
    var prochain_x = 0;
   	var prochain_y = 0;
    if (chemin.length > 0) {
        prochain_x = chemin[0].y + 1;
       	prochain_y = chemin[0].x + 1;
        debug_astar(chemin);
    } else {
        // No path found => exploration
        var nearest_explorable_cell = find_nearest_explorable_cell();
        prochain_x = nearest_explorable_cell.x;
        prochain_y = nearest_explorable_cell.y;
    }

   	if (prochain_x > robot_x) {
   		return "D";
   	}
   	if (prochain_x < robot_x) {
   		return "G";
   	}
   	if (prochain_y > robot_y) {
   		return "H";
   	}
   	if (prochain_y < robot_y) {
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
