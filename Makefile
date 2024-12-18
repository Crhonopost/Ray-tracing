# Makefile pour un unique ex�cutable

# liste des variables � renseigner
#   CIBLE : nom du programme ( $(CIBLE).c doit contenir main() )
#   SRCS : ensemble des fichiers sources 
#   LIBS : liste des biblioth�ques utiles � l'�dition des liens 
#          (format : -lnom1 -lnom2 ...) 
#   PREFIX : chemin de la hi�rarchie 
#
# NE PAS OUBLIER D'AJOUTER LA LISTE DES DEPENDANCES A LA FIN DU FICHIER

CIBLE = main
SRCS =  src/Camera.cpp main.cpp src/Trackball.cpp src/imageLoader.cpp src/Mesh.cpp src/AccelerationStruct.cpp
LIBS =  -lglut -lGLU -lGL -lm -lpthread 
#########################################################"

INCDIR = ./include
LIBDIR = .
BINDIR = .
BUILDDIR = ./build

# nom du compilateur
CC = g++
CPP = g++

CFLAGS = -g -Wall -O0
CXXFLAGS = -g -Wall -O0


# option du preprocesseur
CPPFLAGS =  -I$(INCDIR) 

# options du linker et liste des biblioth�ques � charger
LDFLAGS = -L/usr/X11R6/lib              
LDLIBS = -L$(LIBDIR) $(LIBS)  

# construire la liste des fichiers objets une nouvelle chaine � partir
# de SRCS en substituant les occurences de ".c" par ".o" 
# OBJS = $(SRCS:.cpp=.o)
OBJS = $(patsubst %.cpp, $(BUILDDIR)/%.o, $(SRCS))

# cible par d�faut
# $(CIBLE): $(OBJS)
$(CIBLE): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $(CIBLE)

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

install:  $(CIBLE)
	cp $(CIBLE) $(BINDIR)/

installdirs:
	test -d $(INCDIR) || mkdir $(INCDIR)
	test -d $(LIBDIR) || mkdir $(LIBDIR)
	test -d $(BINDIR) || mkdir $(BINDIR)

clean:
	rm -f  *~  $(CIBLE) $(OBJS)

veryclean: clean
	rm -f $(BINDIR)/$(CIBLE)

dep:
	gcc $(CPPFLAGS) -MM $(SRCS)

# liste des d�pendances g�n�r�e par 'make dep'
Camera.o: src/Camera.cpp include/Camera.h include/Vec3.h include/Trackball.h
main.o: main.cpp include/Vec3.h include/Camera.h include/Trackball.h
Trackball.o: src/Trackball.cpp include/Trackball.h
AccelerationStruct.o : src/AccelerationStruct.cpp include/AccelerationStruct.h
