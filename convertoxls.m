data=load('fcpv');
 f=fieldnames(data);
 for k=1:size(f,1)
   xlswrite('finalresults.xlsx',data.(f{k}),f{k})
 end