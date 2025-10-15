% this function is used to calculate the area of the inductor
function Area = calculate_area(geomParam,Param)

numPolyEdge = Param.polyEdge;% polygon edges
widthSpace = geomParam(:,2);% line space
widthLineMax = geomParam(:,1);% max line width
widthLineMin = geomParam(:,3);% min line width
widthInnerPoly = geomParam(:,5);% innermost polygon width
numTurns = geomParam(:,4);% turns
numDR = geomParam(:,6);% the deformation ratio
widthLineMax(widthLineMax==widthLineMin,:) = widthLineMin(widthLineMax==widthLineMin,:)+0.001;

if Param.indType == 1
    widthLineDelta = (widthLineMax-widthLineMin)./(numTurns.*numPolyEdge);
    innerDiaHeight = numDR.*widthInnerPoly + (widthLineMin+widthSpace).*0.5;
    innerDiaWidth = widthInnerPoly + widthLineMin + widthSpace;
    for ii = 1:length(widthLineDelta)
        widthLine  = widthLineMin(ii):widthLineDelta(ii):widthLineMax(ii);
        outerDiaHeight(ii,:) = innerDiaHeight(ii) + sum(widthLine(1:numPolyEdge/2:end)) + (length(widthLine(1:numPolyEdge/2:end))-2)*widthSpace(ii);
        outerDiaWidth(ii,:) = innerDiaWidth(ii) + sum(widthLine(1+numPolyEdge/4:numPolyEdge/2:end)) + (length(widthLine(1+numPolyEdge/4:numPolyEdge/2:end))-2)*widthSpace(ii);
    end
elseif Param.indType == 2
    widthLineDelta = (widthLineMax-widthLineMin)./(numTurns-1);
    innerDiaHeight = numDR.*widthInnerPoly;
    innerDiaWidth = widthInnerPoly;
    for ii = 1:length(widthLineDelta)
        if numTurns(ii) == 1
            widthLine = (widthLineMax(ii)+widthLineMin(ii))/2;
            outerDiaHeight(ii,:) = innerDiaHeight(ii) + 2*widthLine;
            outerDiaWidth(ii,:) = innerDiaWidth(ii) + 2*widthLine;
        else
            widthLine  = widthLineMin(ii):widthLineDelta(ii):widthLineMax(ii);
            outerDiaHeight(ii,:) = innerDiaHeight(ii) + sum(widthLine)*2 + (numTurns(ii)-1)*2*widthSpace(ii);
            outerDiaWidth(ii,:) = innerDiaWidth(ii) + sum(widthLine)*2 + (numTurns(ii)-1)*2*widthSpace(ii);
        end
    end
end

Area.innerDiaHeight = innerDiaHeight;
Area.innerDiaWidth = innerDiaWidth;
Area.outerDiaHeight = outerDiaHeight;
Area.outerDiaWidth = outerDiaWidth;
Area.area = outerDiaHeight.*outerDiaWidth;
end