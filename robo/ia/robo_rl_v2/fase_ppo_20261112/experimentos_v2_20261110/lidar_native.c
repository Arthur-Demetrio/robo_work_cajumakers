#include <math.h>
#include <stdint.h>
void cast(const uint8_t *cells,int width,int height,double resolution,double floor_w,double floor_h,double ox,double oy,const double *directions,int beams,const double *distances,int samples,double max_range,double *result){
 for(int b=0;b<beams;b++){
  result[b]=max_range;
  for(int k=0;k<samples;k++){
   const double x=ox+directions[2*b]*distances[k];
   const double y=oy+directions[2*b+1]*distances[k];
   if(x<0 || y<0 || x>=floor_w || y>=floor_h)continue;
   int ix=(int)floor(x/resolution),iy=(int)floor(y/resolution);
   if(ix>=0 && iy>=0 && ix<width && iy<height && cells[iy*width+ix]){result[b]=distances[k];break;}
  }
 }
}
