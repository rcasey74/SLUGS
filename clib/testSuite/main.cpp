#include "catch_default_main.hpp" // This brings in a default implementation for main()
#include "circBuffer.h"


  TEST_CASE( "Anadir y quitar elementos", "La cola debe de anadir y remover elementos de manera adecuada" )
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

        x=readHead(bp);
        printf("\ncabeza=%d\n",x);
        z=readFront(bp);
        x=readHead(bp);
        printf("\ncabeza=%d\n",x);
        printf("\nfrente=%d\n",z);

        z=readFront(bp);
        printf("\nfrente=%d\n",z);
  	  REQUIRE( peak(bp) == 4 ); //regresa  el valor de la  cabeza  si  el  buffer  no  esta  vacio.
      REQUIRE( getLength(bp) == 4 );


      // Quitar un elemento
     REQUIRE(readHead(bp) == 0 );
     x = readFront(bp);  //lee  la  cabeza  del  bufer  circular y actualiza  el  valor  de la cabeza siempre  y  cuando  no  haya desbordamiento.
     writeBack(bp, 8);
     REQUIRE( x == 5 );

      REQUIRE( peak(bp) == 8 );

      REQUIRE( getLength(bp) == 1);

      REQUIRE( getOverflow(bp) == 0 );

//      freeCircBuffer(bp)
  }
