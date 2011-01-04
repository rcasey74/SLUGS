#include "catch_default_main.hpp" // This brings in a default implementation for main()
#include "../circBuffer.h"


  TEST_CASE( "Anadir y quitar elementos", "La cola debe de anadir y remover elementos de manera adecuada" )
  {
  		CBRef bp = newCircBuffer(10);
  		int x;
  	
  		// Anadir un elemento
	    writeBack(bp, 5);  		
  		REQUIRE( peak(bp) == 5 );
      REQUIRE( getLength(bp) == 1 );
      
      
      // Quitar un elemento
	    x = readHead(bp);
      REQUIRE( x == 5 );
      REQUIRE( getLength(bp) == 0 );
      
      freeCircBuffer(bp)
  }