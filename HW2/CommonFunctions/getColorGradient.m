function colorTriplet = getColorGradient(pt,totPts,lightTriplet,darkTriplet)

%+ Select RGB color triplet bounds using uisetcolor offline, by inputting
% those bounds, a total number of points, and an intermediate point, return
% the intermediate color triplet; 
%+ Revision List: 
%+ Rev 1.0: Initial Release

colorTriplet = lightTriplet + (darkTriplet-lightTriplet)*(pt-1)/(totPts-1); 

% % Test Code

% lightBlue = [.67 .78 1];
% darkBlue = [0 .17 .51]; 
% 
% lightGreen = [.74 1 .78];
% darkGreen = [0 .48 .06];
% 
% lightRed = [1 .66 .66] ;
% darkRed = [.52 0 0];
% lightBlack = [.91 .91 .91]; 
% darkBlack = [0 0 0];
% totPts = 4; 
% figure(2);clf
% figure(3);clf
% figure(4);clf
% for i = 1:totPts
%     figure(2)
%     colorTriplet = getColorGradient(i,totPts,lightGreen,darkGreen);
%     plot([0 1],[i i],'Color',colorTriplet,'LineWidth',10)
%     hold on
% 
%     figure(3)
%     colorTriplet = getColorGradient(i,totPts,lightRed,darkRed);
%     plot([0 1],[i i],'Color',colorTriplet,'LineWidth',10)
%     hold on
% 
%     figure(4)
%     colorTriplet = getColorGradient(i,totPts,lightBlue,darkBlue);
%     plot([0 1],[i i],'Color',colorTriplet,'LineWidth',10)
%     hold on
% end
end