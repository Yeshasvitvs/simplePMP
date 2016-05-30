function sum = gammaInt(gamma,n,RAMP_KONSTANT)

  k=1;a=0; 
  
  sum=gamma(1,1);
  c=2; h=1;
  
  while(k < n-1)
      fk=gamma(1+k,1);
      c=6-c;
      sum=sum+(c*fk);
      k=k+1;
  end
  
  sum = (RAMP_KONSTANT*sum)/3;
  
end