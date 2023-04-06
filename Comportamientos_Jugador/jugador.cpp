#include "../Comportamientos_Jugador/jugador.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <map>
using namespace std;

#define PI 3.14159265

Action ComportamientoJugador::think(Sensores sensores){
	Action accion = actIDLE;

	// Imprimimos datos.

	printStatus(sensores);

	// Rellenamos información del entorno. 

	if (!sensores.reset) {
		updateStatus(sensores);
	} else {

		if (bien_situado)
			updateLossMap(Pos2D(current_state.fil, current_state.col)); 

		bien_situado = false;
		
		resetVirtualMap();

		current_state.fil = current_state.col = 1.5*size;
		current_state.brujula = norte;

		bikini_on = shoes_on = false;
	}

	if ((sensores.terreno[0] == 'G' and !bien_situado) or sensores.nivel == 0) { // Casilla de posicionamiento o bien en nivel 0. 
		bien_situado = true;

		Pos2D actual_pos(sensores.posF, sensores.posC);
		updateRealMap(actual_pos);
		updateLossMap(actual_pos);

		resetVirtualMap();
		realToVirtualMap(sensores);
		resetLossMap();

		current_state.fil = sensores.posF;
		current_state.col = sensores.posC;
		current_state.brujula = sensores.sentido;
	} else if (sensores.terreno[0] == 'K' && !bikini_on) {
		bikini_on = true;
	} else if (sensores.terreno[0] == 'D' && !shoes_on) {
		shoes_on = true;
	}

	fillVirtualMap(sensores);

	if(bien_situado) {
		fillViewingMap(sensores);
	} 
	
	// 4

	Orientacion preferred_direction = bestDirection(sensores);
	accion = getAction(preferred_direction);

	// if (accion == actFORWARD) // Dejamos rastro de función de pérdida. 
		fillViewingLoss();
	
	last_action = accion;

	return accion;
}

int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}

void ComportamientoJugador::fillViewingMap(const Sensores &sensors) {
	for (int i=0; i<=NUM_SEEN; ++i) {
		Pos2D new_pos = findNewBox(i, current_state.brujula);

		mapaResultado[new_pos.x][new_pos.y] = sensors.terreno[i];
	}
}

void ComportamientoJugador::fillVirtualMap(const Sensores &sensors) {
	for (int i=0; i<=NUM_SEEN; ++i) {
		Pos2D new_pos = findNewBox(i, current_state.brujula);
		
		if (virtual_map[new_pos.x][new_pos.y].type == '?')
			virtual_map[new_pos.x][new_pos.y] = Box(sensors.terreno[i], 0);
	}
}

Pos2D ComportamientoJugador::findNewBox(int num, Orientacion ori) {
	Pos2D box;

	Pos2D displacement = findRelativeDisplacement(num, ori);

	box.x = current_state.fil + displacement.x;
	box.y = current_state.col + displacement.y;

	return box;
}

Pos2D ComportamientoJugador::findRelativeDisplacement(int num, Orientacion ori) {
	Pos2D pos;

	int front = 0;
	int swift = 0;

	if (ori == norte || ori == este || ori == sur || ori == oeste) {
		
		// Based on relative displacement.  
		
		int n = 1;

		while (num >= n*n) {
			++n;
		}

		front = n-1;
		swift = -1 * front + (num - front * front);

		switch(ori) {
			case norte:
				pos.x = -front;
				pos.y = +swift;
				break;
			case este:
				pos.x = +swift;
				pos.y = +front;
				break;
			case sur:
				pos.x = +front;
				pos.y = -swift;
				break;
			case oeste:
				pos.x = -swift;
				pos.y = -front;
				break;
		}
	} else if (ori == noreste || ori == sureste || ori == suroeste || ori == noroeste) {
		
		// Based on relative position.

		int n = 1;

		while (num >= n*n) {
			++n;
		}

		int n_previous = n-1;
		int mid_point = n * n_previous; // Stands for approx: (n-1)^2 + (n^2 - (n-1)^2)/2. 0.5 is negligible. 

		if (num <= mid_point) {
			front = n_previous;
			swift = n_previous - (mid_point - num);

		} else {
			front = n_previous - (num - mid_point);
			swift = n_previous;
		}

		switch (ori) {
			case noreste:
				pos.x = -front;
				pos.y = +swift;
				break;
			case sureste:
				pos.x = +swift;
				pos.y = +front;
				break;
			case suroeste:
				pos.x = +front;
				pos.y = -swift;
				break;
			case noroeste:
				pos.x = -swift;
				pos.y = -front;
				break;
		}
	}
	
	return pos;
}

Orientacion ComportamientoJugador::bestDirection(const Sensores &sensores) {
	Orientacion orientation = norte;
	Pos2D best_pos(current_state.fil-1, current_state.col);
	int min_loss = numeric_limits<int>::max();

	for (int i=0; i<8; ++i) {
		float angle = (2 * PI * i) / 8;

		int horizontal = round(sin(angle));
		int vertical = round(cos(angle)); 

		Pos2D new_pos;

		new_pos.x = current_state.fil - vertical;
		new_pos.y = current_state.col + horizontal;

		Orientacion local_orientation = static_cast<Orientacion>(i);
		int loss = getBoxLoss(new_pos, local_orientation, sensores.bateria);

		if (local_orientation == current_state.brujula && 
			(sensores.superficie[1] == 'l' || sensores.superficie[2] == 'l' || sensores.superficie[3] == 'l' || sensores.superficie[1] == 'a' || sensores.superficie[2] == 'a' || sensores.superficie[3] == 'a')
			)
			loss = 1e4;

		if (loss < min_loss) {
			min_loss = loss;
			best_pos = new_pos;
			orientation = local_orientation;
		}
	}

	int num_unreachable = 0;
	Orientacion most_unknown = mostUnknownDirection(num_unreachable);
	int pos = static_cast<int>(most_unknown);

	float angle = (2 * PI * pos) / 8;

	int horizontal = round(sin(angle));
	int vertical = round(cos(angle)); 

	Pos2D new_pos;

	new_pos.x = current_state.fil - vertical;
	new_pos.y = current_state.col + horizontal;

	if (getBoxLoss(new_pos, most_unknown, sensores.bateria) < FLEXITY_FACTOR*getBoxLoss(best_pos, orientation, sensores.bateria) && num_unreachable > 0) {
		orientation = most_unknown;
		best_pos = new_pos;
	}
	

	return orientation;
}

Orientacion ComportamientoJugador::mostUnknownDirection(int &num_unreachable) {
	Orientacion best_orientation = norte;
	int max_unknown = 0;
	int depth = size/2;

	for (int i=0; i<8; ++i) {
		Orientacion ori = static_cast<Orientacion>(i);
		int count = 0;

		count = countReachableUnknownBoxes(depth, ori);

		if (count > max_unknown) {
			max_unknown = count;
			best_orientation = ori;
		}
	}

	num_unreachable = max_unknown;
	return best_orientation;
}

int ComportamientoJugador::countReachableUnknownBoxes(int max_depth, Orientacion ori) {
	int count = 0;
	bool stop = false;

	int considered_size = (bien_situado ? size : virtual_map.size());

	/*
	for (int depth=0; depth<max_depth && !stop; ++depth) {
		int ini = depth*depth;
		int fin = (depth+1)*(depth+1);

		for (int i=ini; i<fin; ++i) {
			Pos2D new_pos = findNewBox(i, ori);

			if (0<= min(new_pos.x, new_pos.y) && max(new_pos.x, new_pos.y) < considered_size) {
				unsigned char type = virtual_map[new_pos.x][new_pos.y].type;

				if (type == '?')
					++count;
				else if (type == 'P')
					stop = true;
			} else {
				stop = true;
			}
			
		}
		
	}
	*/

	int ori_number = static_cast<int>(ori);
	float angle = (2 * PI * ori_number) / 8;

	int horizontal = round(sin(angle));
	int vertical = round(cos(angle));

	for (int i=0; i<max_depth && !stop; ++i) {
		Pos2D new_pos;

		new_pos.x = current_state.fil - i*vertical;
		new_pos.y = current_state.col + i*horizontal;

		if (0<= min(new_pos.x, new_pos.y) && max(new_pos.x, new_pos.y) < considered_size) {
			unsigned char type = virtual_map[new_pos.x][new_pos.y].type;

			switch(type) {
				case '?':
					++count;
					break;
				case 'M':
				case 'P':
					stop = true;
					break;
				default:
					break;
			}
		} else {
			stop = true;
		}
	} 

	return count;
}

int ComportamientoJugador::getBoxLoss(Pos2D pos, Orientacion orientation, int battery_level) {
	int result = 0;

	unsigned char type = virtual_map[pos.x][pos.y].type;
	
	int the_base_loss = 0;
	float the_loss = 0;
	int the_rotation_loss = 0;
	
	// Ajustamos pérdida base.

	the_base_loss = loss_base[type];

	if (type == 'B' && shoes_on)
		the_base_loss = 28;
	else if (type == 'A' && bikini_on)
		the_base_loss = 30;
	else if (type == 'G' && bien_situado)
		the_base_loss = 5;
	else if (type == 'D' && shoes_on)
		the_base_loss = 5;
	else if (type == 'K' && bikini_on)
		the_base_loss = 5;

	// Ajustamos pérdida relativa. 

	for (int i=0; i<virtual_map[pos.x][pos.y].rel_loss; ++i)
		the_loss += ADD_FACTOR;
	
	if (the_loss > MAX_REL_LOSS)
		the_loss = MAX_REL_LOSS;

	// Ajustamos pérdida por rotación. 

	Action action = getAction(orientation);
	unsigned char current_type = virtual_map[pos.x][pos.y].type;

	switch (action) {
		case actFORWARD:

			switch(current_type) {
				case 'A':
					the_rotation_loss = (bikini_on ? 8 : 15);
					break;
				case 'B':
					the_rotation_loss = (shoes_on ? 10 : 13);
					break;
				case 'T':
					the_rotation_loss = 1;
					break;
				default:
					the_rotation_loss = 0;
					break;
			}
			break;
		case actTURN_BL:
		case actTURN_BR:
			switch(current_type) {
				case 'A':
					the_rotation_loss = (bikini_on ? 27 : 33);
					break;
				case 'B':
					the_rotation_loss = (shoes_on ? 18 : 23);
					break;
				case 'T':
					the_rotation_loss = 12;
					break;
				default:
					the_rotation_loss = 11;
					break;
			}
			break;
		case actTURN_SL:
		case actTURN_SR:
			switch(current_type) {
				case 'A':
					the_rotation_loss = (bikini_on ? 27 : 40);
					break;
				case 'B':
					the_rotation_loss = (shoes_on ? 18 : 23);
					break;
				case 'T':
					the_rotation_loss = 17;
					break;
				default:
					the_rotation_loss = 16;
					break;
			}
			break;
		default:
			the_rotation_loss = 0;
			break;
	}

	result = the_base_loss + the_loss + the_rotation_loss;

	return result;
}

void ComportamientoJugador::fillViewingLoss() {

	Pos2D new_pos = findNewBox(1, current_state.brujula);

	
	for (int i=0; i<NUM_SEEN; ++i) {
		Pos2D new_pos = findNewBox(i, current_state.brujula);
		int depth = max(abs(new_pos.x - current_state.fil), abs(new_pos.y - current_state.col));

		virtual_map[new_pos.x][new_pos.y].rel_loss += (MAX_DEPTH - depth);
	}
}

Action ComportamientoJugador::getAction(Orientacion orientation) {
	Action action = actIDLE;

	int rel_diff = (static_cast<int>(orientation) - static_cast<int>(current_state.brujula) + 8) % 8;

	switch (rel_diff) {
		case 0:
			action = actFORWARD;
			break;
		case 1:
		case 2:
			action = actTURN_SR;
			break;
		case 3:
			action = actTURN_BR;
			break;
		case 4:
		case 5:
			action = actTURN_BL;
			break;
		case 6:
		case 7:
			action = actTURN_SL;
			break;
		default:
			action = actIDLE;
			break;
	}

	return action;
}

void ComportamientoJugador::printStatus(const Sensores &sensores) {
	cout << "Posicion: fila " << sensores.posF << " columna " << sensores.posC << " ";

	switch(sensores.sentido){
		case 0: cout << "Norte" << endl; break;
		case 1: cout << "Noreste" << endl; break;
		case 2: cout << "Este" << endl; break;
		case 3: cout << "Sureste" << endl; break;
		case 4: cout << "Sur " << endl; break;
		case 5: cout << "Suroeste" << endl; break;
		case 6: cout << "Oeste" << endl; break;
		case 7: cout << "Noroeste" << endl; break;
	}
	cout << "Terreno: ";
	for (int i=0; i<sensores.terreno.size(); i++)
		cout << sensores.terreno[i];
	cout << endl;

	cout << "Superficie: ";
	for (int i=0; i<sensores.superficie.size(); i++)
		cout << sensores.superficie[i];
	cout << endl;

	cout << "Colisión: " << sensores.colision << endl;
	cout << "Reset: " << sensores.reset << endl;
	cout << "Vida: " << sensores.vida << endl;
	cout << endl;
}

void ComportamientoJugador::updateStatus(const Sensores &sensores) {

	if (sensores.colision)
		return;

	if (sensores.nivel != 0) {
		int a;

		switch(last_action) {
			case actFORWARD:
				switch(current_state.brujula){
					case norte: 
						current_state.fil--; 
						break;
					case noreste: 
						current_state.fil--; 
						current_state.col++; 

						break;
					case este: 
						current_state.col++; 
						break;
					case sureste: 
						current_state.fil++; 
						current_state.col++; 
						break;
					case sur: 
						current_state.fil++; 
						break;
					case suroeste: 
						current_state.fil++; 
						current_state.col--; 
						break;
					case oeste: 
						current_state.col--; 
						break;
					case noroeste: 
						current_state.fil--; 
						current_state.col--; 
						break;
				}
				break;
			case actTURN_SL:
				a = current_state.brujula;
				a = (a+7) % 8;
				current_state.brujula = static_cast<Orientacion>(a);
				break;
			case actTURN_SR:
				a = current_state.brujula;
				a = (a+1) % 8;
				current_state.brujula = static_cast<Orientacion>(a);
				break;
			case actTURN_BL:
				a = current_state.brujula;
				a = (a+5) % 8;
				current_state.brujula = static_cast<Orientacion>(a);
				break;
			case actTURN_BR:
				a = current_state.brujula;
				a = (a+3) % 8;
				current_state.brujula = static_cast<Orientacion>(a);
				break;
		}
	} else {
		current_state.fil = sensores.posF;
		current_state.col = sensores.posC;
		current_state.brujula = sensores.sentido;
	}

	
}

void ComportamientoJugador::updateRealMap(Pos2D actual_pos) {
	Pos2D displacement;

	displacement.x = actual_pos.x - current_state.fil;
	displacement.y = actual_pos.y - current_state.col;

	for (int i=0; i<virtual_map.size(); ++i) {
		for (int j=0; j<virtual_map[i].size(); ++j) {
			if (virtual_map[i][j].type != '?') {
				mapaResultado[i+displacement.x][j+displacement.y] = virtual_map[i][j].type;
			}
		}
	}
}

void ComportamientoJugador::updateLossMap(Pos2D actual_pos) {
	if (!bien_situado)
		return;
	
	Pos2D displacement;

	displacement.x = actual_pos.x - current_state.fil;
	displacement.y = actual_pos.y - current_state.col;

	for (int i=0; i<virtual_map.size(); ++i) {
		for (int j=0; j<virtual_map[i].size(); ++j) {
			if (virtual_map[i][j].type != '?') {
				loss_map[i+displacement.x][j+displacement.y] += virtual_map[i][j].rel_loss;
			}
		}
	}
}

void ComportamientoJugador::realToVirtualMap(const Sensores &sensores) {
	resetVirtualMap();

	for (int i=0; i<size; ++i) {
		for (int j=0; j<size; ++j) {
			virtual_map[i][j] = Box(mapaResultado[i][j], loss_map[i][j]);
		}
	}

	current_state.fil = sensores.posF;
	current_state.col = sensores.posC;
	current_state.brujula = sensores.sentido;
}

void ComportamientoJugador::resetVirtualMap() {
	Box null_box('?', 0);

	for (int i=0; i<virtual_map.size(); ++i) {
		for (int j=0; j<virtual_map[i].size(); ++j) {
			virtual_map[i][j] = null_box;
		}
	}
}

void ComportamientoJugador::resetLossMap() {
	for (int i=0; i<loss_map.size(); ++i) {
		for (int j=0; j<loss_map[i].size(); ++j) {
			loss_map[i][j] = 0;
		}
	}
}