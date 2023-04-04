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

  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size){

      // Constructor de la clase
      // Dar el valor inicial a las variables de estado
      last_action = actIDLE;
      
      //Orientacion = norte;
      current_state.fil = current_state.col = size;
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

      loss_base['B'] = 20;
      loss_base['A'] = 25;
      loss_base['P'] = 1e5;
      loss_base['S'] = 5;
      loss_base['T'] = 6;
      loss_base['M'] = 1e5;
      loss_base['K'] = 0;
      loss_base['D'] = 0;
      loss_base['X'] = 1;
      loss_base['G'] = 0;
      loss_base['?'] = 1;

      // Creamos un mapa propio.

      for (int i=0; i<2*size; ++i) {
        vector<Box> line(2*size, Box('?', 0));
        virtual_map.push_back(line);
      }
    }

    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);

  private:

  static const int MAX_DEPTH = 4;
  static const int NUM_SEEN = MAX_DEPTH * MAX_DEPTH - 1;

  static const int MAX_REL_LOSS = 900;
  const float MULT_FACTOR = 1.25;
  
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


  // Métodos privados

  void printStatus(const Sensores &sensors);
  void updateStatus(const Sensores &sensors);

  void fillViewingMap(const Sensores &sensors);
  Pos2D findRelativeDisplacement(int num);
  Orientacion bestDirection(const Sensores &sensors);

  int getBoxLoss(Pos2D pos, Orientacion orientation);
  void fillViewingLoss();

  Action getAction(Orientacion orientation);

  void fillVirtualMap(const Sensores &sensors);
  void realToVirtualMap(const Sensores &sensors);
  void resetVirtualMap();

  void updateRealMap(const Sensores &sensors);
  
  void updateLossMap(const Sensores &sensors);
  void resetLossMap();
};

#endif
