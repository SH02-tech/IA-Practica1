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
		bien_situado = false;
		
		updateLossMap(sensores);
		resetVirtualMap();

		current_state.fil = current_state.col = size;
		current_state.brujula = norte;

		bikini_on = shoes_on = false;
	}

	if (sensores.terreno[0] == 'G' and !bien_situado) { // Casilla de posicionamiento. 
		bien_situado = true;

		updateRealMap(sensores);
		
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

	if (accion == actFORWARD) // Dejamos rastro de función de pérdida. 
		fillViewingLoss();
	
	last_action = accion;

	return accion;
}

int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}

void ComportamientoJugador::fillViewingMap(const Sensores &sensors) {
	for (int i=0; i<=NUM_SEEN; ++i) {
		Pos2D displacement = findRelativeDisplacement(i);
		
		int new_x = current_state.fil + displacement.x;
		int new_y = current_state.col + displacement.y;

		mapaResultado[new_x][new_y] = sensors.terreno[i];
	}
}

void ComportamientoJugador::fillVirtualMap(const Sensores &sensors) {
	for (int i=0; i<=NUM_SEEN; ++i) {
		Pos2D displacement = findRelativeDisplacement(i);

		int new_x = current_state.fil + displacement.x;
		int new_y = current_state.col + displacement.y;
		
		if (virtual_map[new_x][new_y].type == '?')
			virtual_map[new_x][new_y] = Box(sensors.terreno[i], 0);
	}
}

Pos2D ComportamientoJugador::findRelativeDisplacement(int num) {
	Pos2D pos;

	int front = 0;
	int swift = 0;

	Orientacion ori = current_state.brujula;

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
	int min_loss = numeric_limits<int>::max();

	for (int i=0; i<8; ++i) {
		float angle = (2 * PI * i) / 8;

		int horizontal = round(sin(angle));
		int vertical = round(cos(angle)); 

		Pos2D new_pos;

		new_pos.x = current_state.fil - vertical;
		new_pos.y = current_state.col + horizontal;

		Orientacion local_orientation = static_cast<Orientacion>(i);
		int loss = getBoxLoss(new_pos, local_orientation);

		if (local_orientation == current_state.brujula && (sensores.superficie[1] == 'l' || sensores.superficie[1] == 'a'))
			loss = 1e4;

		if (loss < min_loss) {
			min_loss = loss;
			orientation = local_orientation;
		}
	}

	return orientation;
}

int ComportamientoJugador::getBoxLoss(Pos2D pos, Orientacion orientation) {
	int result = 0;

	unsigned char type = virtual_map[pos.x][pos.y].type;
	
	int the_base_loss = 0;
	float the_loss = 1.0;
	int the_rotation_loss = 0;
	
	// Ajustamos pérdida base.

	the_base_loss = loss_base[type];

	if (type == 'B' && shoes_on)
		the_base_loss = 10;
	else if (type == 'A' && bikini_on)
		the_base_loss = 15;
	else if (type == 'G' && bien_situado)
		the_base_loss = 5;
	else if (type == 'D' && shoes_on)
		the_base_loss = 5;
	else if (type == 'K' && bikini_on)
		the_base_loss = 5;

	// Ajustamos pérdida relativa. 

	for (int i=0; i<virtual_map[pos.x][pos.y].rel_loss; ++i)
		the_loss *= MULT_FACTOR;
	
	if (the_loss > MAX_REL_LOSS)
		the_loss = MAX_REL_LOSS;

	// Ajustamos pérdida por rotación. 

	Action action = getAction(orientation);
	unsigned char current_type = virtual_map[pos.x][pos.y].type;

	switch (action) {
		case actFORWARD:

			switch(current_type) {
				case 'A':
					the_rotation_loss = (bikini_on ? 10 : 200);
					break;
				case 'B':
					the_rotation_loss = (shoes_on ? 15 : 100);
					break;
				case 'T':
					the_rotation_loss = 2;
					break;
				default:
					the_rotation_loss = 1;
					break;
			}
			break;
		case actTURN_BL:
		case actTURN_BR:
			switch(current_type) {
				case 'A':
					the_rotation_loss = (bikini_on ? 15 : 250);
					break;
				case 'B':
					the_rotation_loss = (shoes_on ? 16 : 103);
					break;
				case 'T':
					the_rotation_loss = 4;
					break;
				default:
					the_rotation_loss = 2;
					break;
			}
			break;
		case actTURN_SL:
		case actTURN_SR:
			switch(current_type) {
				case 'A':
					the_rotation_loss = (bikini_on ? 15 : 700);
					break;
				case 'B':
					the_rotation_loss = (shoes_on ? 16 : 103);
					break;
				case 'T':
					the_rotation_loss = 4;
					break;
				default:
					the_rotation_loss = 2;
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

	virtual_map[current_state.fil][current_state.col].rel_loss += 3;

	for (int i=1; i<NUM_SEEN; ++i) {
		Pos2D displacement = findRelativeDisplacement(i);

		int new_x = current_state.fil + displacement.x;
		int new_y = current_state.col + displacement.y;

		virtual_map[new_x][new_y].rel_loss++;
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
			action = actTURN_SR;
			break;
		case 2:
		case 3:
			action = actTURN_BR;
			break;
		case 4:
		case 5:
		case 6:
			action = actTURN_BL;
			break;
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

void ComportamientoJugador::updateRealMap(const Sensores &sensors) {
	Pos2D displacement;

	displacement.x = sensors.posF - current_state.fil;
	displacement.y = sensors.posC - current_state.col;

	for (int i=0; i<virtual_map.size(); ++i) {
		for (int j=0; j<virtual_map[i].size(); ++j) {
			if (virtual_map[i][j].type != '?') {
				mapaResultado[i+displacement.x][j+displacement.y] = virtual_map[i][j].type;
			}
		}
	}
}

void ComportamientoJugador::updateLossMap(const Sensores &sensors) {
	if (!bien_situado)
		return;
	
	
	Pos2D displacement;

	displacement.x = sensors.posF - current_state.fil;
	displacement.y = sensors.posC - current_state.col;

	for (int i=0; i<virtual_map.size(); ++i) {
		for (int j=0; j<virtual_map[i].size(); ++j) {
			if (virtual_map[i][j].type != '?') {
				loss_map[i+displacement.x][j+displacement.y] = virtual_map[i][j].rel_loss;
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