import sys
import random

TOTAL = 100

mapa = sys.argv[1]
sem = sys.argv[2]
level = int(sys.argv[3])
tam = int(sys.argv[4])

ori = 0


f = open("bateria.sh", "w")

for i in range(TOTAL):
	x_ini = random.randint(3, tam-3)
	y_ini = random.randint(3, tam-3)
	
	if level == 0:
		ori = random.randint(0, 7)
	
	f.write(f"printf \"Caso {i} ((x,y) = ({x_ini}, {y_ini}):  \\n---------------------------------------\\n \"")
	f.write("\n")
	f.write(f"./practica1SG {mapa} {sem} {level} {x_ini} {y_ini} {ori} | tail -n 2")
	f.write("\n")
	f.write("printf \"\\n\"")
	f.write("\n\n")
