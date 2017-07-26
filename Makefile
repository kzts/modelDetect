DIR = src/
SRC = gpio.c
OBJ = $(DIR)$(SRC:.c=.o)
NAME = ./lib/libgpiommap.a
TMS = main.c
TMO = $(TMS:.c=.o)
TMN = gpiommap
RM = rm -f

XENO = /usr/xenomai
XENOCONFIG=$(shell PATH=$(XENO):$(XENO)/bin:$(PATH) which xeno-config 2>/dev/null)
CC = $(shell $(XENOCONFIG) --cc)
CFLAGS   = -Wextra -g -MMD -MP $(shell $(XENOCONFIG) --xeno-cflags) $(MY_CFLAGS)
LDFLAGS  = $(shell $(XENOCONFIG) --xeno-ldflags) $(MY_LDFLAGS) -lnative
LDFLAGS+=-Xlinker -rpath -Xlinker $(shell $(XENOCONFIG) --libdir)

#remoteControl:remoteControl.c
#	$(CC) $(CFLAGS) -o remoteControl -Isrc remoteControl.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers

liball: $(OBJ)
	ar -rc $(NAME) $(OBJ)
	ranlib $(NAME)
d:
	$(CC) $(CFLAGS) -o modelDetect -Isrc modelDetect.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers
a:
	$(CC) $(CFLAGS) -o moveArm -Isrc moveArm.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers
s:
	$(CC) $(CFLAGS) -o swing -Isrc swing.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers
c:
	$(CC) $(CFLAGS) -o controlAngle -Isrc controlAngle.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers

#remoteControl:remoteControl.c 
#	$(CC) $(CFLAGS) -o remoteControl -Isrc remoteControl.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers
#xen:
#	$(CC) $(CFLAGS) -o remoteControl -Isrc remoteControl.c enc.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers
#	$(CC) $(CFLAGS) -o remoteControl -Isrc remoteControl.c -lgpiommap $(LDFLAGS) -O2 -Wno-missing-field-initializers

clean:
	-$(RM) $(OBJ)
	-$(RM) *~
	-$(RM) \#*
	-$(RM) *.core

fclean: clean
	-$(RM) $(NAME)
	-$(RM) $(TMN)

re: fclean liball

$(NAME): liball

ftest: fclean test

install: liball
	cp src/gpio.h /usr/include/
	cp src/am335x.h /usr/include/
	cp lib/libgpiommap.a /usr/lib/
