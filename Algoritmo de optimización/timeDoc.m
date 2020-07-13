function [] = timeDoc(t_final, delta)
%--------------------------ARCHIVO DE TIEMPO-------------------------------
    time=[0.1:delta:t_final].';
    final=zeros(size(time));
    final(end,1)=1;
    content=[0 0;
             0.001 0;
             0.002 0;
             0.005 0;
             time final];
    fileID = fopen('tiempo.csv','w');
    fprintf(fileID,'%4s,%5s\n','time','final');
    fprintf(fileID,'%3.3f,%2.0f\n',content.');
    fclose(fileID);
end