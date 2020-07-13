function [] = ecuacionesDoc(wayi, wayf)
    xi= num2str(wayi(1));
    yi= num2str(wayi(2));
    zi= num2str(wayi(3));
    xf= num2str(wayf(1));
    yf= num2str(wayf(2));
    zf= num2str(wayf(3));
    stringx = [' x = ' xi];
    stringx = [stringx blanks(30-size(stringx,2)-15) '#Pos. Inicial x'];
    stringy = [' y = ' yi];
    stringy = [stringy blanks(30-size(stringy,2)-15) '#Pos. Inicial y'];
    stringz = [' z = ' zi];
    stringz = [stringz blanks(30-size(stringz,2)-15) '#Pos. Inicial z'];
    stringxf = [' xf = ' xf];
    stringxf = [stringxf blanks(29-size(stringxf,2)-13) '#Pos. Final x'];
    stringyf = [' yf = ' yf];
    stringyf = [stringyf blanks(29-size(stringyf,2)-13) '#Pos. Final y'];
    stringzf = [' zf = ' zf];
    stringzf = [stringzf blanks(29-size(stringzf,2)-13) '#Pos. Final z'];

    fileID = fopen('ecuaciones.apm','r+');

    fseek(fileID, 1065, 'bof');% Condiciones iniciales x, y, z
    fprintf(fileID,'%30s\n',stringx);
    fprintf(fileID,'%30s\n',stringy);
    fprintf(fileID,'%30s\n',stringz);

    fseek(fileID, 904, 'bof');% Condiciones finales UAV
    fprintf(fileID,'%29s\n',stringxf);
    fprintf(fileID,'%29s\n',stringyf);
    fprintf(fileID,'%29s\n',stringzf);

    fclose(fileID);
end