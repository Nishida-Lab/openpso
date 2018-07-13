double my_clock2()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec+(double)tv.tv_usec*1e-6; 
}
