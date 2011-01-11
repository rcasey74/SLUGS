#include "catch_default_main.hpp" // This brings in a default implementation for main()
#include "circBuffer.h"
#include "../conversions.h"

TEST_CASE("Peak muestra el proximo elemento a retirar", "Peak debe de mostrar el proximo elemento que readFront nos daria"){

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


TEST_CASE(" Introducir datos y mostrar cuantos elementos hay en el bufer", "debe  mostrar cuantos  elementos  hay en el  buffer"){

    struct CircBuffer buffer;
    int x,z;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    REQUIRE( getLength(bp) == NULL);

    for (x=0; x<200; x++)
    writeBack(bp, 5);

    REQUIRE( readTail(bp) == 200);

}


TEST_CASE(" Obtener cuantos elementos hay en el bufer", "getLength  debe  mostrar cuantos  elementos  hay en el  buffer"){

    struct CircBuffer buffer;
    int x;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    REQUIRE( getLength(bp) == NULL);


    for (x=0; x<200; x++)
    writeBack(bp, 5);

    x=getLength(bp);
     printf("elementos en el bufer : %d\n",x);
    REQUIRE( x == 200);
}



TEST_CASE(" Pedir el  frente sin tener algo en el bufer", "que valor  entrega si no hay elementos en el buffer"){

    struct CircBuffer buffer;
    int x;

    CBRef bp = &buffer;
    newCircBuffer(bp);

    x=readFront(bp);

    REQUIRE(x==128);

}



TEST_CASE("REALIZAR  UN  OVERFLOW EN EL  BUFER", " getOverflow   debe  mostrar si  hay  desbordamiento  en el bufer"){

    struct CircBuffer buffer;
    int x,j;


    CBRef bp = &buffer;
    newCircBuffer(bp);


   for (x=0; x<550; x++)
    writeBack(bp, 5);

   j=getOverflow(bp);
   printf("elementos desbordados :%d  \n",j);

    REQUIRE( getOverflow(bp) != 0);

    REQUIRE( getOverflow(bp) == 39);


}


TEST_CASE("Vaciar   el  bufer", " makeEmpty  debe  si  efectivamente el bufer se  vacia"){

    struct CircBuffer buffer;
    int x,f,n;


    CBRef bp = &buffer;
    newCircBuffer(bp);


   for (x=0; x<480; x++)
    writeBack(bp, 5);

   n=readTail(bp);
   printf("elementos  en el  bufer: %d\n",n);

   makeEmpty(bp);

   REQUIRE( readTail(bp)== 0);

}


TEST_CASE("Leer  del  bufer  sin  haber  introducido datos", "Que  sucede si  se  pide datos  en el  bufer  vacio"){

    struct CircBuffer buffer;
    int x;


    CBRef bp = &buffer;
    newCircBuffer(bp);

    x=peak(bp);


   REQUIRE( x  == 0);

}


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

        REQUIRE(getLength(bp) == 4);

        z=readFront(bp);//marca  al  byte  como leido  y  pasa la cabeza al  siguiente elemento.
        printf("\nfrente=%d\n",z);

        z=readFront(bp);
        printf("\nfrente=%d\n",z);
  	  REQUIRE( peak(bp) == 6 ); //regresa  el valor de la  cabeza  si  el  buffer  no  esta  vacio.
      REQUIRE( getLength(bp) == 2 );


      // Quitar un elemento
     REQUIRE(readHead(bp) == 2 );
     x = readFront(bp);  //lee  la  cabeza  del  bufer  circular y actualiza  el  valor  de la cabeza siempre  y  cuando  no  haya desbordamiento.
     writeBack(bp, 8);
      REQUIRE( x == 6 );

      REQUIRE( getLength(bp) == 2);

//      freeCircBuffer(bp)
  }

TEST_CASE("Vaciar   el  bufer", " makeEmpty  debe  si  efectivamente el bufer se  vacia"){

    struct CircBuffer buffer;
    int x;


    CBRef bp = &buffer;
    newCircBuffer(bp);


   for (x=0; x<480; x++)
    writeBack(bp, 5);

    //checar lo de  la liberacion del  bufer.

   REQUIRE( readTail(bp)== 480);

}

//  test  del  archivo  de  conversiones   //

TEST_CASE("Conversion de  Bytes a   float", " Checar si se  realiza  efectivamente  la  conversion de Bytes a float"){


    //unsigned char x[4]={1,0,0,0};
    int  n,t,r,ce,te,se,pe,res;
    unsigned char z[2]={99,0};
    unsigned char c;
    float f,flo;
    unsigned short int s;     //valor maximo   0   a  65535

     unsigned char x;
     flo=0.345;
      x=23;
    //c=200;


   f = bytesToFloat(&x);

    //f=floatToBytes( flo, x);


    printf("%.2f\n",f);

}
    //REQUIRE( f == 50.00);

/*
///////////////////////////////////////////////////
    s=bytesToShort(z);

    printf("%d\n",s);

   REQUIRE( s == 99);



///////////////////////////////////////////

//procedimiento  para el  ver los  indices del intercambio  de  informacion
// de bytes to  Short.


printf("\nteclea un numero\n");
scanf("%d",&n);

t=n/256;
r=n%256;

z[0]=r;
z[1]=t;

s=bytesToShort(z);
printf("%d\n",s);


//REQUIRE( bytesToShort(z)== n);
/*
unsigned char b[4]={1,0,0,0};
int  n;
float f;

    f = bytesToFloat(b);

    printf("%.2f\n",f);

    //REQUIRE( f == 0.00);

}


TEST_CASE("Conversion de  short bytes a UShort", " Checar si se  realiza  efectivamente  la  conversion short bytes a UShort"){

unsigned char x;
unsigned short s;
float f;

    x=0;

    s = bytesToUShort(&x);



}





TEST_CASE("Conversion de  Short  a Bytes", " Checar si se  realiza  efectivamente  la  conversion de Short  a  Bytes"){

unsigned char x;
short s;



    shortToBytes(s, &x);


    //REQUIRE( f == 0);

}

*/

