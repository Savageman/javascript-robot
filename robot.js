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

/*

Queue.js

A function to represent a queue

Created by Stephen Morley - http://code.stephenmorley.org/ - and released under
the terms of the CC0 1.0 Universal legal code:

http://creativecommons.org/publicdomain/zero/1.0/legalcode

*/

/* Creates a new queue. A queue is a first-in-first-out (FIFO) data structure -
 * items are added to the end of the queue and removed from the front.
 */
function Queue(){

  // initialise the queue and offset
  var queue  = [];
  var offset = 0;

  // Returns the length of the queue.
  this.getLength = function(){
    return (queue.length - offset);
  }

  // Returns true if the queue is empty, and false otherwise.
  this.isEmpty = function(){
    return (queue.length == 0);
  }

  /* Enqueues the specified item. The parameter is:
   *
   * item - the item to enqueue
   */
  this.enqueue = function(item){
    queue.push(item);
  }

  /* Dequeues an item and returns it. If the queue is empty, the value
   * 'undefined' is returned.
   */
  this.dequeue = function(){

    // if the queue is empty, return immediately
    if (queue.length == 0) return undefined;

    // store the item at the front of the queue
    var item = queue[offset];

    // increment the offset and remove the free space if necessary
    if (++ offset * 2 >= queue.length){
      queue  = queue.slice(offset);
      offset = 0;
    }

    // return the dequeued item
    return item;

  }

  /* Returns the item at the front of the queue (without dequeuing it). If the
   * queue is empty then undefined is returned.
   */
  this.peek = function(){
    return (queue.length > 0 ? queue[offset] : undefined);
  }

  /* Returns the index of an element */
  this.indexOf = function(lookup) {
      var idx = queue.indexOf(lookup, offset) - offset;
      return (idx < 0 ? -1 : idx);
  }

  this.get = function(index) {
      return queue[index + offset];
  }

  this.put = function(index, val) {
      queue[index + offset] = val;
  }

  this.reset = function() {
      while(!this.isEmpty()) {
          this.dequeue();
      }
  }

  this.log = function() {
      console.log('length = ' + this.getLength() +
              ' offset = ' + offset +
              ' queue = ' + queue);
  }
}
/* END Queue.js */


// pcc = plus court chemin
// tableau dijkstra
var pcc = new Array();
var pcc_mark = new Array();
var liste_stations = new Array();
var cell_queue = new Queue();
var cell_exists = new Queue();

function init_custom() {
	for (j = hauteur_terrain; j >= 1; j--) {
		pcc[j] = new Array();
		for (i = 1; i <= largeur_terrain; i++) {
			pcc[j][i] = 99;
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
	//console.log(y, x, "=", min);
	dijkstra_bfs_maj(y, x, min + 1, true);
	return true;
}



/* Ajoute une case à la file d'attente */
function addToQueue(y, x, dist) {
    /* If node has not already been visited */
    if(pcc_mark[y][x] == false ) {
        /* If it is not already on the list */
        var cell = cell_exists.indexOf(y + '_' + x);

        if(cell == -1) {
            //console.log('Adding to queue ' + y + '/' + x);
            cell_exists.enqueue(y + '_' + x);
            cell_queue.enqueue([y, x, dist]);
        } else {
            var  val = cell_queue.get(cell);
            if(val[2] > dist) {
                //console.log('Updating in queue ' + y + '/' + x);
                val[2] = dist;
                cell_queue.put(cell, val);
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
        //console.log(cell_queue.getLength());
        /* The working node */
        //cell_queue.log();
        var step = cell_queue.dequeue();
        cell_exists.dequeue();
        //cell_queue.log();

        /* The node is updated by the value */
        var texp = terrain_explore[step[0]][step[1]];
        if(texp != 1 && texp != 9) {
            /* If no value in cell */
            if((pcc[step[0]][step[1]] < 99 /*&& !onlyUnknown*/) 
                    || (pcc[step[0]][step[1]] == 99)) {
                /* If dist is lower */
                console.log('Updating ' + step[0] + '/' + step[1] + ' to ' + step[2] + ' from ' + pcc[step[0]][step[1]] + ' to ' + step[2]);
                if(pcc[step[0]][step[1]] > step[2]) {
                    pcc[step[0]][step[1]] = step[2];
                }
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
}

function decider_direction(CH, CB, CG, CD, but_dir_hb, but_dir_gd, but_dist, reserve_carburant) {

	count_recursive = 0;
	for (station in liste_stations) {
		dijkstra_bfs_maj(liste_stations[station][0], liste_stations[station][1], 0, true);
	}
	
	//console.log('maj station', count_recursive);
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
	//console.log('add station', count_recursive);

	// Mise à jour case courante
	//dijkstra_maj_unknown_cell(robot_y, robot_x);
    dijkstra_bfs_maj_ukn(robot_y, robot_x);
    
	//console.log('robot', count_recursive);

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
