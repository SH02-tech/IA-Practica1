#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"
#include <map>
using namespace std;

struct state{
  int fil;
  int col;
  Orientacion brujula;
};

struct Box {
  unsigned char type;
  int rel_loss;

  Box() {
    type = '?';
    rel_loss = 0;
  }

  Box(unsigned char the_type, int the_loss) {
    type = the_type;
    rel_loss = the_loss;
  }
};

struct Pos2D {
  int x, y;

  Pos2D() {
    x = y = 0;
  }

  Pos2D(int the_x, int the_y) {
    x = the_x;
    y = the_y;
  }
};


class ComportamientoJugador : public Comportamiento{
  private: // Atributos
    static const int MAX_DEPTH = 4;
    static const int NUM_SEEN = MAX_DEPTH * MAX_DEPTH - 1;

    static const int MAX_REL_LOSS = 920;
    const float ADD_FACTOR = 0.75;
    const float FLEXITY_FACTOR = 1.7;
    
    // Declarar aquí las variables de estado
    Action last_action;
    // Orientacion brujula.
    state current_state;

    bool bikini_on, shoes_on;
    
    bool bien_situado;

    vector<vector<Box>> virtual_map;
    vector<vector<int>> loss_map;

    map<unsigned char, int> loss_base;

    int size;

  public:

    ComportamientoJugador(unsigned int size) : Comportamiento(size){

      // Constructor de la clase
      // Dar el valor inicial a las variables de estado
      last_action = actIDLE;
      
      //Orientacion = norte;
      current_state.fil = current_state.col = 1.5*size;
      current_state.brujula = norte;

      bien_situado = false;
      
      bikini_on = false;
      shoes_on = false;

      this->size = size;

      // Creamos el sistema de puntuaciones

      for (int i=0; i<size; ++i) {
        vector<int> linea(size, 0);
        loss_map.push_back(linea);
      }

      loss_base['B'] = 35;
      loss_base['A'] = 100;
      loss_base['P'] = 1e5;
      loss_base['S'] = 5;
      loss_base['T'] = 6;
      loss_base['M'] = 1e5;
      loss_base['K'] = -20;
      loss_base['D'] = -20;
      loss_base['X'] = 5;
      loss_base['G'] = -20;
      loss_base['?'] = 2;

      // Creamos un mapa propio.

      for (int i=0; i<3*size; ++i) {
        vector<Box> line(3*size, Box('?', 0));
        virtual_map.push_back(line);
      }
    }

    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);

  private: // Métodos privados

    // Status

    void printStatus(const Sensores &sensors);
    void updateStatus(const Sensores &sensors);

    // Mapa Resultados

    void fillViewingMap(const Sensores &sensors);
    void updateRealMap(Pos2D actual_pos);
    Pos2D findNewBox(int num, Orientacion ori);
    Pos2D findRelativeDisplacement(int num, Orientacion ori);

    // Orientation decision. 

    Orientacion bestDirection(const Sensores &sensors);
    Orientacion mostUnknownDirection(int &num_unreachable);
    int countReachableUnknownBoxes(int max_depth, Orientacion ori);

    Action getAction(Orientacion orientation);

    // Mapa Loss

    int getBoxLoss(Pos2D pos, Orientacion orientation, int battery_level);
    void fillViewingLoss();

    // Mapa Virtual

    void fillVirtualMap(const Sensores &sensors);
    void realToVirtualMap(const Sensores &sensors);
    void resetVirtualMap();
    void updateLossMap(Pos2D actual_pos);
    void resetLossMap();
};

#endif
