
CHIP=atmega644p
FCPU=12000000 

OUTPUT=libiob.a

SOURCES=iob.c

OBJECTS=$(SOURCES:.c=.o)



CC=avr-gcc
AR=avr-ar

CFLAGS=-mmcu=$(CHIP) -DF_CPU=$(FCPU) -Os
ARFLAGS=-crs

all: $(OUTPUT)

$(OUTPUT): $(OBJECTS) 
	$(AR) $(ARFLAGS) $(OUTPUT) $(OBJECTS)

%.o: %.c 
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY:
clean:
	rm -f *.o $(OUTPUT)
	