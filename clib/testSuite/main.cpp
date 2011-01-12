#include "catch_default_main.hpp" // This brings in a default implementation for main()
#include "circBuffer.h"
#include "../conversions.h"

TEST_CASE("circBuffer", "Tests requeridos para cada funcion de Circular Buffer"){

    struct CircBuffer buffer;
    CBRef bp = &buffer;
    newCircBuffer(bp);

    SECTION ("newCircBuffer()", "Debe inicializar todo a cero"){
        REQUIRE(getLength(bp) == 0);
        REQUIRE(readTail(bp) == 0);
        REQUIRE(readHead(bp) == 0);
        REQUIRE(getOverflow(bp) == 0);
    }

//////  nuevas  secciones //////


SECTION ("Peak muestra el proximo elemento a retirar"  , "Peak debe de mostrar el proximo elemento que readFront nos daria "){
    struct CircBuffer buffer;
    int x,y;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    writeBack(bp, 5);
    REQUIRE(peak(bp) == 5);

    writeBack(bp, 2);
    REQUIRE(peak(bp) == 5);
    x = readFront(bp);
    REQUIRE_FALSE(peak(bp) == 6);
    REQUIRE(peak(bp) == 2);

    writeBack(bp, 6);
    REQUIRE(peak(bp) == 2);
    x = readFront(bp);
    REQUIRE(peak(bp) == 6);
    x = readFront(bp);
    REQUIRE_FALSE(peak(bp) == 6);
    REQUIRE(peak(bp) == 0);
    }


SECTION (" Introducir datos y mostrar cuantos elementos hay en el bufer", "debe  mostrar cuantos  elementos  hay en el  buffer"){
    struct CircBuffer buffer;
    int x,z;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    REQUIRE( getLength(bp) == 0);

    for (x=0; x<200; x++){
        writeBack(bp, 5);
    }

    readFront(bp);
    writeBack(bp, 5);


    REQUIRE( getLength(bp) == 200);
    }


SECTION ( " Obtener cuantos elementos hay en el bufer", "getLength  debe  mostrar cuantos  elementos  hay en el  buffer"){
     struct CircBuffer buffer;
    int x;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    REQUIRE( getLength(bp) == NULL);


    for (x=0; x<200; x++)
    writeBack(bp, 5);

    x=getLength(bp);

    REQUIRE( x == 200);

    }


SECTION ( " Pedir el  frente sin tener algo en el bufer", "que valor  entrega si no hay elementos en el buffer"  ){

    struct CircBuffer buffer;
    int x;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    x=readFront(bp);

    REQUIRE(x==128);


    }

SECTION ( "REALIZAR  UN  OVERFLOW EN EL  BUFER", " getOverflow   debe  mostrar si  hay  desbordamiento  en el bufer"  ){

    struct CircBuffer buffer;
    int x,j;


    CBRef bp = &buffer;
    newCircBuffer(bp);


   for (x=0; x<550; x++)
    writeBack(bp, 5);

   j=getOverflow(bp);


    REQUIRE( getOverflow(bp) != 0);

    REQUIRE( getOverflow(bp) == 39);

    }


SECTION ("Vaciar   el  bufer", " makeEmpty  debe  si  efectivamente el bufer se  vacia"  ){

    struct CircBuffer buffer;
    int x,f,n;


    CBRef bp = &buffer;
    newCircBuffer(bp);


   for (x=0; x<480; x++)
    writeBack(bp, 5);

   n=readTail(bp);

   makeEmpty(bp);

   REQUIRE(getLength(bp) == 0);
   REQUIRE(readTail(bp) == 0);
   REQUIRE(readHead(bp) == 0);
   REQUIRE(getOverflow(bp) == 0);


    }


SECTION ("Leer  del  bufer  sin  haber  introducido datos", "Que  sucede si  se  pide datos  en el  bufer  vacio"    ){
    struct CircBuffer buffer;
    int x;


    CBRef bp = &buffer;
    newCircBuffer(bp);

    x=peak(bp);


    REQUIRE( x  == 0);
    }



SECTION ( "Anadir y quitar elementos", "La cola debe de anadir y remover elementos de manera adecuada" )
  {
      struct CircBuffer buffer;

  		CBRef bp = &buffer;
  		newCircBuffer(bp);

  		int x,i,z;

  		// Anadir un elemento
	    writeBack(bp, 5);  //escribe  un  byte  al final del  buffer circular  e incrementa  el contador de desbordamiento  si esto  ocurre
        writeBack(bp, 2);
        writeBack(bp, 6);
        writeBack(bp, 9);

        REQUIRE(getLength(bp) == 4);

        z=readFront(bp);//marca  al  byte  como leido  y  pasa la cabeza al  siguiente elemento.

        z=readFront(bp);

  	  REQUIRE( peak(bp) == 6 ); //regresa  el valor de la  cabeza  si  el  buffer  no  esta  vacio.
      REQUIRE( getLength(bp) == 2 );


      // Quitar un elemento
     REQUIRE(readHead(bp) == 2 );
     x = readFront(bp);  //lee  la  cabeza  del  bufer  circular y actualiza  el  valor  de la cabeza siempre  y  cuando  no  haya desbordamiento.
     writeBack(bp, 8);
      REQUIRE( x == 6 );

      REQUIRE( getLength(bp) == 2);

  }



SECTION ("Leer  la  cola del  bufer", " Checar si  la  cola apunta  correctamente en el buffer "){

    struct CircBuffer buffer;
    int x,r;


    CBRef bp = &buffer;
    newCircBuffer(bp);


   for (x=0; x<480; x++)
    writeBack(bp, 5);


   REQUIRE( readTail(bp)== 480);

}

SECTION ("funcion freebuffer", " Checar que freebuffer libera el buffer "){
int  x;
    struct CircBuffer buffer;
    CBRef bp = &buffer;

     for (x=0; x<380; x++)
     writeBack(bp, 3);

     REQUIRE( bp != NULL );

     freeCircBuffer(&bp);


     REQUIRE( bp == NULL );

}


///hasta  aqui  de  circBuffer
}

TEST_CASE("conversions", "Tests requeridos para cada funcion de conversions.c"){

    union floatToChar {
        float fl ;
        unsigned char ch[4];
    };

    union floatToChar temp;


    union CharToUShort {
        unsigned short   UShr;
        unsigned char ch[2];
    };

    union CharToUShort tmpUsh;


    union ShorToChar {
    short   		 shD;
    unsigned char    ch[2];
 	};

    union   ShorToChar  tshch;

    SECTION ("floatToBytes()", "La conversion de flotante a bytes para transmision"){

        temp.fl = 50.0;

        printf("%f => [%d][%d][%d][%d]\n", temp.fl, temp.ch[0], temp.ch[1], temp.ch[2], temp.ch[3]);

        REQUIRE(bytesToFloat(temp.ch) == temp.fl);

        temp.fl = 0.0;
    }



SECTION ("bytesToFloat()", "conversion de bytes a  flotante para transmision"){

        temp.ch[0] = 0;
        temp.ch[1] =0;
        temp.ch[2] =200;
        temp.ch[3] =65;

        temp.fl=bytesToFloat(temp.ch);

        REQUIRE(temp.fl == bytesToFloat(temp.ch)  );

        //temp.fl = 0.0;
    }



SECTION ("BytesToUShort()", "La conversion de Bytes a UShort para transmision"){


        tmpUsh.UShr = 33;

        REQUIRE(bytesToShort(tmpUsh.ch) == tmpUsh.UShr);

        tmpUsh.UShr = 0;


    }



SECTION ("UShortToBytes()", "La conversion de UShort  a   bytes para transmision"){

        tmpUsh.ch[0] = 244;

        tmpUsh.ch[1] = 1;

        REQUIRE(500 == bytesToShort(tmpUsh.ch) );

        tmpUsh.UShr = 0;
    }


SECTION ("ShorToChar()", "La conversion de Short  a   bytes para transmision"){

        tshch.shD=303;

        REQUIRE( tshch.shD == bytesToShort(tshch.ch) );

        tshch.shD = 0;
    }


SECTION ("CharToShort()", "La conversion de Bytes  a  Short  para transmision"){

        tshch.ch[0]=179;

        tshch.ch[1]=4;

        REQUIRE( tshch.shD == 1203 );

        tshch.shD = 0;
    }


////hasta aqui  temina  el test  de  las  conversiones.
}




