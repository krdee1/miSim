function p = okabeIto()
    % Okabe & Ito (2008) colorblind-safe palette, minus yellow (too light on
    % white paper). Returns an N-by-3 RGB matrix, e.g. colororder(okabeIto()).
    p = [  0, 114, 178;   % blue
         213,  94,   0;   % vermillion
           0, 158, 115;   % bluish green
         204, 121, 167;   % reddish purple
          86, 180, 233;   % sky blue
         230, 159,   0;   % orange
           0,   0,   0] / 255;
end
