/*
* définition du terrain
*/
largeur_terrain = 25;
hauteur_terrain = 16;


/*
* Légende du terrain :
* 0 = terrain vide
* 1 = obstacle
* 2 = but
* 3 = ressource
* 4 = position du robot
* (attention : la case but et la case de départ du robot sont données plus loin)
*/
terrain = new Array();
terrain[16] = new Array(0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 0, 0);
terrain[15] = new Array(0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 3, 3, 3, 3, 3, 0, 0, 3);
terrain[14] = new Array(0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 3, 3, 3, 0, 0, 0, 0, 0);
terrain[13] = new Array(0, 0, 0, 1, 0, 0, 1, 3, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
terrain[12] = new Array(0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 3, 1, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
terrain[11] = new Array(0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
terrain[10] = new Array(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
terrain[9] =  new Array(0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0);
terrain[8] =  new Array(0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 3, 0, 1, 1, 0, 0);
terrain[7] =  new Array(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 3, 1, 1, 0, 0, 1, 1, 0);
terrain[6] =  new Array(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0);
terrain[5] =  new Array(0, 0, 3, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0);
terrain[4] =  new Array(0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 3, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0);
terrain[3] =  new Array(0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
terrain[2] =  new Array(0, 0, 0, 1, 0, 0, 0, 3, 1, 0, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
terrain[1] =  new Array(0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//                            ^
but_x = 2;
but_y = 15;

robot_x = 25;
robot_y = 16;
