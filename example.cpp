#include <stdio.h>


/* Les deux bibliothèques nécessaires d'opencv :
	- cv.h contient les structures et fonctions de manipulation d'images
	- highgui.h contient les fonctions d'affichage des images
*/
#include <cv.h>
#include <highgui.h>

int main(int argc, char *argv[])
{
  /* On initialise la 'capture' depuis la WebCam, une adresse,
     représentative de la ressource est retournée
   */
  CvCapture* capture = cvCaptureFromCAM(-1);
  if(capture) // Si la caméra est reconnu
  {
	if(!cvGrabFrame(capture)) // On prend une image et une seule !
	{
  		printf("Could not grab a frame\n\7"); // Si la prise d'image n'est pas possible, on sort !
  		exit(0);
	}
  }
  else // Si la caméra n'est pas reconnu ou si elle n'existe pas, on sort...
  {
  	printf("Could not open video device\n");
  	exit(0);
  }
	IplImage *img=cvRetrieveFrame(capture); // On rapatrie l'image que l'on 'stocke' dans img avec ses propriétés.

  printf("L'image fait %dx%d pixels et possède %d canaux (couleurs)\n",img->width,img->height,img->nChannels);

  // On crée une fenètre dans laquelle on affichera l'image
  cvNamedWindow("Fenetre_test", CV_WINDOW_AUTOSIZE);

  // C'est ce que l'on fait ici :
  cvShowImage("Fenetre_test", img );

  // Appuyez sur une touche pour sortir
  cvWaitKey(0);

  // On libère ensuite, la mémoire de l'image et de la ressource
  cvReleaseImage(&img );
  cvReleaseCapture(&capture);
  return 0;
}
