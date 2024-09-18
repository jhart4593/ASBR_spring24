function [xsol,fval,history] = runfmincon(objfun,x0,A,b,Aeq,beq,lb,ub,confun)
% Reference: https://www.mathworks.com/help/optim/ug/output-functions.html

plotflag = 0; 
% Set up shared variables with outfun
history.x = [];
history.fval = [];
% searchdir = [];
% Call optimization
options = optimoptions(@fmincon,'OutputFcn',@outfun,... 
    'Display','iter','Algorithm','active-set');
% % Call optimization
% options = optimoptions(@fmincon,'OutputFcn',@outfun,... 
%     'Display','iter','Algorithm','interior-point');

% options = optimoptions(@fmincon,'OutputFcn',@outfun);
[xsol,fval] = fmincon(objfun,x0,A,b,Aeq,beq,lb,ub,confun,options);

function stop = outfun(x,optimValues,state)
    stop = false;

    switch state
        case 'init'
            hold on
        case 'iter'
            % Concatenate current point and objective function
            % value with history. x must be a row vector.
            history.fval = [history.fval; optimValues.fval];
            history.x = [history.x; x'];
            %          % Concatenate current search direction with
            %          % searchdir.
            %            searchdir = [searchdir;...
            %                         optimValues.searchdirection'];
            if plotflag
                plot(x(1),x(2),'o');
                % Label points with iteration number and add title.
                % Add .15 to x(1) to separate label from plotted 'o'.
                text(x(1)+.15,x(2),...
                    num2str(optimValues.iteration));
                title('Sequence of Points Computed by fmincon');
            end
        case 'done'
            hold off
        otherwise
    end
end
 
end