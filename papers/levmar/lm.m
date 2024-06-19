function [a,redX2,sigma_a,sigma_y,corr_a,R_sq,cvg_hst] = lm(func,a,t,y_dat,weight,da,a_lb,a_ub,c,opts)
% [a,redX2,sigma_a,sigma_y,corr_a,R_sq,cvg_hst] = lm(func,a,t,y_dat,weight,da,a_lb,a_ub,c,opts)
%
% Levenberg Marquardt curve-fitting: minimize sum of weighted squared residuals
% ----------  INPUT  VARIABLES  -----------
% func   = function of n independent variables, 't', and m coefficients, 'a', 
%          returning the simulated model: y_hat = func(t,a,c)
% a      = initial guess of coefficient values                             (n x 1)
% t      = independent variables (used as arg to func)                   (m x 1)
% y_dat  = data to be fit by func(t,p)                                   (m x 1)
% weight = weights or a scalar weight value ( weight >= 0 ) ...          (m x 1)
%          inverse of the standard measurement errors
%          Default:  ( 1 / ( y_dat' * y_dat ))
% da     = fractional increment of 'a' for numerical derivatives
%          da(j)>0 central differences calculated
%          da(j)<0 one sided 'backwards' differences calculated
%          da(j)=0 sets corresponding partials to zero; i.e. holds a(j) fixed
%          Default:  0.001;
% a_lb   = lower bounds for coefficient values                           (n x 1)
% a_ub   = upper bounds for coefficient values                           (n x 1)
% c      = an optional set of model constants passed to y_hat = func(t,a,c)
% opts   = vector of algorithmic parameters 
%             parameter defaults     meaning
% opts(1)  =  prnt            3         >1 intermediate results; >2 plots
% opts(2)  =  MaxEvals        10*Ncof^2 maximum number of function calls
% opts(3)  =  epsilon_1       1e-3      convergence tolerance for gradient
% opts(4)  =  epsilon_2       1e-3      convergence tolerance for coefficients
% opts(5)  =  epsilon_3       1e-1      convergence tolerance for red. Chi-sqr
% opts(6)  =  epsilon_4       1e-1      determines acceptance of a L-M step
% opts(7)  =  lambda_0        1e-2      initial value of L-M paramter
% opts(8)  =  lambda_UP_fac   11        factor for increasing lambda
% opts(9)  =  lambda_DN_fac    9        factor for decreasing lambda
% opts(10) =  Update_Type      1        1: Levenberg-Marquardt lambda update
%                                       2: Quadratic update 
%                                       3: Nielsen's lambda update equations
%
% ----------  OUTPUT  VARIABLES  -----------
% a       = least-squares optimal estimate of the coefficient values
% redX2   = reduced Chi squared error criteria - should be close to 1
% sigma_a = asymptotic standard error of the coefficients
% sigma_y = asymptotic standard error of the curve-fit
% corr_a  = correlation matrix of the coefficients
% R_sq    = R-squared cofficient of multiple determination  
% cvg_hst = convergence history ... see lm_plots.m
 
%   Henri Gavin, Dept. Civil & Environ. Engineering, Duke Univ. 4 May 2016
%   modified from: http://octave.sourceforge.net/optim/function/leasqr.html
%   using references by
%   Press, et al., Numerical Recipes, Cambridge Univ. Press, 1992, Chapter 15.
%   Sam Roweis       http://www.cs.toronto.edu/~roweis/notes/lm.pdf
%   Manolis Lourakis http://www.ics.forth.gr/~lourakis/levmar/levmar.pdf
%   Hans Nielson     http://www2.imm.dtu.dk/~hbn/publ/TR9905.ps
%   Mathworks        optimization toolbox reference manual
%   K. Madsen, H.B., Nielsen, and O. Tingleff
%   http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf

 global   iteration  func_calls

 tensor_coefficient = 0;                  % set to 1 of coefficient is a tensor

 iteration  = 0;                        % iteration counter
 func_calls = 0;                        % running count of function evaluations

%p = a(:); y_dat = y_dat(:); t = t(:);  % make column vectors
 Ncof   = length(a);                    % number of coefficients
 Npnt   = length(y_dat);                % number of data points
 a_old  = zeros(Ncof,1);                % previous set of coefficients
 y_old  = zeros(Npnt,1);                % previous model, y_old = y_hat(t;a_old)
 X2     = 1e-3/eps;                     % a really big initial Chi-sq value
 X2_old = 1e-3/eps;                     % a really big initial Chi-sq value
 J      = zeros(Npnt,Ncof);             % Jacobian matrix
 DoF    = Npnt - Ncof;                  % statistical degrees of freedom


 if size(t,1) ~= Npnt
    disp('lm.m error: the number of rows of t must equal the length of y_dat');
    rows_t = size(t,1)
    length_y_dat = length(y_dat)
    X2 = 0; corr_a = 0; sigma_a = 0; sigma_y = 0; R_sq = 0; cvg_hist = 0;
    if ~tensor_coefficient, 
         return;                
    end
 end

 if nargin <  5, weight = 1/(y_dat'*y_dat); end
 if nargin <  6, da = 0.001; end
 if nargin <  7, a_lb   = -100*abs(a); end
 if nargin <  8, a_ub   =  100*abs(a); end
 if nargin <  9, c       =  1; end
 if nargin < 10,                % Algorithmic Paramters
%         prnt MaxEvals eps1  eps2  eps3  eps4  lam0  lamUP lamDN UpdateType 
   opts = [  3,10*Ncof^2, 1e-3, 1e-3, 1e-1, 1e-1, 1e-2,    11,    9,        1 ];
 end
 prnt          = opts(1);        % >1 intermediate results; >2 plots
 MaxEvals      = opts(2);        % maximum number of function evaluations
 epsilon_1     = opts(3);        % convergence tolerance for gradient
 epsilon_2     = opts(4);        % convergence tolerance for coefficients
 epsilon_3     = opts(5);        % convergence tolerance for Chi-square
 epsilon_4     = opts(6);        % determines acceptance of a L-M step
 lambda_0      = opts(7);        % initial value of damping paramter, lambda
 lambda_UP_fac = opts(8);        % factor for increasing lambda
 lambda_DN_fac = opts(9);        % factor for decreasing lambda
 Update_Type   = opts(10);       % 1: Levenberg-Marquardt lambda update
                                 % 2: Quadratic update 
                                 % 3: Nielsen's lambda update equations

%if ( tensor_coefficient && prnt == 3 ) prnt = 2; end

 plotcmd='figure(102); plot(t(:,1),y_init,''-k'',t(:,1),y_hat,''-b'',t(:,1),y_dat,''o'',''color'',[0,0.6,0],''MarkerSize'',4); title(sprintf(''\\chi^2_\\nu = %f'',X2/DoF)); drawnow ';

 a_lb=a_lb(:); a_ub=a_ub(:); % make column vectors

 if length(da) == 1, da = da*ones(Ncof,1); end

 idx   = find(da ~= 0);            % indices of the coefficients to be fit
 Nfit = length(idx);               % number of coefficients to fit
 stop = 0;                         % termination flag

 y_init = feval(func,t,a,c);       % residual error using a_try

 if ( var(weight) == 0 )           % identical weights vector 
   weight = abs(weight(1))*ones(Npnt,1);        
   disp('using uniform weights for error analysis')
 else
   weight = abs(weight(:));
 end

% initialize Jacobian with finite difference calculation
 [JtWJ,JtWdy,X2,y_hat,J] = lm_matx(func,t,a_old,y_old,1,J,a,y_dat,weight,da,c);

 if ( max(abs(JtWdy)) < epsilon_1 )
   fprintf(' *** Your initial guess meets gradient convergence critira \n')
   fprintf(' *** To converge further, reduce epsilon_1 and restart \n')   
   fprintf(' *** epsilon_1 = %e\n', epsilon_1);
   stop = 1;
 end

 switch Update_Type
   case 1                                 % Marquardt: init'l lambda
     lambda  = lambda_0;
   otherwise                              % Quadratic and Nielsen
     lambda  = lambda_0 * max(diag(JtWJ)); nu=2;
 end

 X2_old = X2;                            % previous value of X2 

 cvg_hst = ones(MaxEvals,Ncof+3);         % initialize convergence history

 while ( ~stop && func_calls <= MaxEvals )        % --- Start Main Loop

   iteration = iteration + 1;
 
% incremental change in coefficients
   switch Update_Type
     case 1                                       % Marquardt
       X = JtWJ + lambda*diag(diag(JtWJ));  
     otherwise                                    % Quadratic and Nielsen
       X = JtWJ + lambda*eye(Ncof); 
   end

   while rcond(X) < 1e-15
     X = X + 1e-6*sum(diag(X))/Ncof*eye(Ncof);
   end

   h = X \ JtWdy;

   if rcond(X) < 1e-14;
      h = 0.1*h;
   end

%  big = max(abs(h./p)) > 2;                      % this is a big step

   % --- Are coefficients [p+h] much better than [p] ?

   a_try = a + h(idx);                            % update the [idx] elements 
   a_try = min(max(a_lb,a_try),a_ub);             % apply bounds

   delta_y = y_dat - feval(func,t,a_try,c);       % residual error using a_try
   if ~all(isfinite(delta_y))                     % floating point error; break
     stop = 1;
     break     
   end
   func_calls = func_calls + 1;
   X2_try = delta_y' * ( delta_y .* weight );     % Chi-squared error criteria

   if ( Update_Type == 2 )                        % Quadratic
%    One step of quadratic line update in the h direction for minimum X2
     alpha =  JtWdy'*h / ( (X2_try - X2)/2 + 2*JtWdy'*h ) ;
     h = alpha * h;

     a_try = a + h(idx);                          % update only [idx] elements
     a_try = min(max(a_lb,a_try),a_ub);           % apply bounds

     delta_y = y_dat - feval(func,t,a_try,c);     % residual error using a_try
     func_calls = func_calls + 1;
     X2_try = delta_y' * ( delta_y .* weight );   % Chi-squared error criteria
   end

   switch Update_Type                             % Nielsen
     case 1
       rho = (X2 - X2_try) / abs( h' * (lambda*diag(diag(JtWJ))*h + JtWdy) );
     otherwise
       rho = (X2 - X2_try) / abs( h' * (lambda * h + JtWdy) );
   end

   if ( rho > epsilon_4 )                         % it IS significantly better

     dX2 = X2 - X2_old;
     X2_old = X2;
     a_old = a;
     y_old = y_hat;
     a = a_try(:);                           % accept a_try

     [JtWJ,JtWdy,X2,y_hat,J] = ...
                       lm_matx(func,t,a_old,y_old,dX2,J,a,y_dat,weight,da,c);

                                % decrease lambda ==> Gauss-Newton method

     switch Update_Type
       case 1                                   % Levenberg
         lambda = max(lambda/lambda_DN_fac,1.e-7);
       case 2                                   % Quadratic
         lambda = max( lambda/(1 + alpha) , 1.e-7 );
       case 3                                   % Nielsen
         lambda = lambda*max( 1/3, 1-(2*rho-1)^3 ); nu = 2;
     end

     if ( prnt > 2 )
       eval(plotcmd);
     end

   else                                           % it IS NOT better

     X2 = X2_old;                             % do not accept a_try

     if ( ~rem(iteration,2*Ncof) )            % rank-1 update of Jacobian
       [JtWJ,JtWdy,dX2,y_hat,J] = ...
                       lm_matx(func,t,a_old,y_old,-1,J,a,y_dat,weight,da,c);
     end

                                % increase lambda  ==> gradient descent method

     switch Update_Type
       case 1                                   % Levenberg
         lambda = min(lambda*lambda_UP_fac,1.e7);
       case 2                                   % Quadratic
         lambda = lambda + abs((X2_try - X2)/2/alpha);
       case 3                                   % Nielsen
         lambda = lambda * nu;   nu = 2*nu;
     end

   end

   if ( prnt > 1 )
     fprintf('>%3d:%3d | chi_sq=%10.3e | lambda=%8.1e \n', ...
                                       iteration,func_calls,X2/DoF,lambda );
     fprintf('      a  :  ');
     for an=1:Ncof
       fprintf(' %10.3e', a(an) );
     end
     fprintf('\n');
     fprintf('    da/a :  ');
     for an=1:Ncof
       fprintf(' %10.3e', h(an) / a(an) );
     end
     fprintf('\n');
   end

% update convergence history ... save _reduced_ Chi-square
   cvg_hst(iteration,:) = [ func_calls  a'  X2/DoF lambda ];


   if ( max(abs(JtWdy)) < epsilon_1  &&  iteration > 2 ) 
     fprintf(' **** Convergence in r.h.s. ("JtWdy")  **** \n')
     fprintf(' **** epsilon_1 = %e\n', epsilon_1);
     stop = 1;
   end
   if ( max(abs(h)./(abs(a)+1e-12)) < epsilon_2  &&  iteration > 2 ) 
     fprintf(' **** Convergence in Parameters **** \n')
     fprintf(' **** epsilon_2 = %e\n', epsilon_2);
     stop = 1;
   end
   if ( X2/DoF < epsilon_3 &&  iteration > 2 ) 
     fprintf(' **** Convergence in reduced Chi-square  **** \n')
     fprintf(' **** epsilon_3 = %e\n', epsilon_3);
     stop = 1;
   end
   if ( func_calls >= MaxEvals )
     disp(' !! Maximum Number of Function Calls Reached Without Convergence !!')
     stop = 1;
   end

 end                                        % --- End of Main Loop

 % --- convergence achieved, find covariance and confidence intervals

% ---- Error Analysis ----

 if var(weight) == 0   % recompute equal weights for paramter error analysis 
   weight = DoF/(delta_y'*delta_y) * ones(Npnt,1);
 end

 if nargout > 1                             % reduced Chi-square
   redX2 = X2 / DoF;
 end

 [JtWJ,JtWdy,X2,y_hat,J] = lm_matx(func,t,a_old,y_old,-1,J,a,y_dat,weight,da,c);

 if nargout > 2                             % standard error of coefficients 
   if rcond(JtWJ) > 1e-15 
     covar_a = inv(JtWJ);
   else
     covar_a = inv(JtWJ + 1e-6*sum(diag(JtWJ))/Ncof*eye(Ncof));
   end
   sigma_a = sqrt(diag(covar_a));
 end

 if nargout > 3                             % standard error of the fit
%  sigma_y = sqrt(diag(J * covar_a * J'));  % slower version of below
   sigma_y = zeros(Npnt,1);
   for i=1:Npnt
     sigma_y(i) = J(i,:) * covar_a * J(i,:)';        
   end
   sigma_y = sqrt(sigma_y);
 end

 if nargout > 4                             % coefficient correlation matrix
   corr_a = covar_a ./ [sigma_a*sigma_a'];        
 end

 if nargout > 5                             % coefficient of multiple determination
   R_sq = corr([y_dat y_hat]);
   R_sq = R_sq(1,2).^2;                
 end

 if nargout > 6                             % convergence history
   cvg_hst = cvg_hst(1:iteration,:);
 end

% endfunction  # ---------------------------------------------------------- LM


function J = lm_FD_J(func,t,a,y,da,c)
% J = lm_FD_J(func,t,a,y,{da},{c})
%
% partial derivatives (Jacobian) dy/da for use with lm.m
% computed via Finite Differences
% Requires n or 2n function evaluations, n = number of nonzero values of da
% -------- INPUT VARIABLES ---------
% func = function of independent variables, 't', and coefficients, 'a',
%        returning the simulated model: y_hat = func(t,a,c)
% t  = independent variables (used as arg to func)                       (m x 1)
% a  = current coefficient values                                          (n x 1)
% y  = func(t,a,c) initialised by user before each call to lm_FD_J       (m x 1)
% da = fractional increment of a for numerical derivatives
%      da(j)>0 central differences calculated
%      da(j)<0 one sided differences calculated
%      da(j)=0 sets corresponding partials to zero; i.e. holds a(j) fixed
%      Default:  0.001;
% c  = an optional set of model constants passed to y_hat = func(t,a,c)
%---------- OUTPUT VARIABLES -------
% J  = Jacobian Matrix J(i,j)=dy(i)/da(j)         i=1:n; j=1:m 

%   Henri Gavin, Dept. Civil & Environ. Engineering, Duke Univ. November 2005
%   modified from: ftp://fly.cnuce.cnr.it/pub/software/octave/leasqr/
%   Press, et al., Numerical Recipes, Cambridge Univ. Press, 1992, Chapter 15.


 global  func_calls

 m=length(y);              % number of data points
 n=length(a);              % number of coefficients

 if nargin < 5
        da = 0.001*ones(1,n);
 end

 as=a; J=zeros(m,n); del=zeros(n,1);         % initialize Jacobian to Zero

 for j=1:n                 % START --- loop over all coefficients

   del(j) = da(j) * (1+abs(a(j)));   % coefficient perturbation
   a(j)   = as(j) + del(j);          % perturb coefficient a(j)

   if del(j) ~= 0
     y1=feval(func,t,a,c);
     func_calls = func_calls + 1;

     if (da(j) < 0)                  % backwards difference
       J(:,j) = (y1-y)./del(j);
     else                            % central difference, additional func call
       a(j) = as(j) - del(j);
       J(:,j) = (y1-feval(func,t,a,c)) ./ (2 .* del(j));
       func_calls = func_calls + 1;
     end
   end

   a(j)=as(j);                       % restore a(j)

 end                       % END --- loop over all coefficients

% endfunction # -------------------------------------------------- LM_FD_J



function J = lm_Broyden_J(a_old,y_old,J,a,y)
% J = lm_Broyden_J(a_old,y_old,J,a,y)
% carry out a rank-1 update to the Jacobian matrix using Broyden's equation
%---------- INPUT VARIABLES -------
% a_old = previous set of coefficients                                     (n x 1)
% y_old = model evaluation at previous set of coefficients, y_hat(t;a_old) (m x 1)
% J  = current version of the Jacobian matrix                            (m x n)
% a     = current  set of coefficients                                     (n x 1)
% y     = model evaluation at current  set of coefficients, y_hat(t;p)     (m x 1)
%---------- OUTPUT VARIABLES -------
% J = rank-1 update to Jacobian Matrix J(i,j)=dy(i)/da(j)  i=1:n; j=1:m  (m x n)

 h  = a - a_old;

 J = J + ( y - y_old - J*h )*h' / (h'*h);       % Broyden rank-1 update eq'n

% endfunction # ---------------------------------------------- LM_Broyden_J



function [JtWJ,JtWdy,Chi_sq,y_hat,J] = lm_matx(func,t,a_old,y_old,dX2,J,a,y_dat,weight,da,c)
% [JtWJ,JtWdy,Chi_sq,y_hat,J] = lm_matx(func,t,a_old,y_old,dX2,J,a,y_dat,weight,{da},{c})
%
% Evaluate the linearized fitting matrix, JtWJ, and vector JtWdy, 
% and calculate the Chi-squared error function, Chi_sq 
% Used by Levenberg-Marquard algorithm, lm.m   
% -------- INPUT VARIABLES ---------
% func   = function of n independent variables, a, and m coefficients, a,
%         returning the simulated model: y_hat = func(t,a,c)
% t      = independent variables (used as arg to func)                   (m x 1)
% a_old  = previous coefficient values                                     (n x 1)
% y_old  = previous model ... y_old = y_hat(t;a_old);                    (m x 1)
% dX2    = previous change in Chi-squared criteria                       (1 x 1)
% J      = Jacobian of model, y_hat, with respect to coefficients, a       (m x n)
% a      = current  coefficient values                                     (n x 1)
% y_dat  = data to be fit by func(t,a,c)                                 (m x 1)
% weight = the weighting vector for least squares fit ...
%          inverse of the squared standard measurement errors
% da     = fractional increment of 'a' for numerical derivatives
%          da(j)>0 central differences calculated
%          da(j)<0 one sided differences calculated
%          da(j)=0 sets corresponding partials to zero; i.e. holds a(j) fixed
%          Default:  0.001;
% c      = an optional set of model constants passed to y_hat = func(t,a,c)
%---------- OUTPUT VARIABLES -------
% JtWJ    = linearized Hessian matrix (inverse of covariance matrix)     (n x n)
% JtWdy   = linearized fitting vector                                    (n x m)
% Chi_sq = Chi-squared criteria: weighted sum of the squared residuals WSSR
% y_hat  = model evaluated with coefficients 'a'                           (m x 1)
% J      = Jacobian of model, y_hat, with respect to coefficients, a       (m x n)

%   Henri Gavin, Dept. Civil & Environ. Engineering, Duke Univ. November 2005
%   modified from: ftp://fly.cnuce.cnr.it/pub/software/octave/leasqr/
%   Press, et al., Numerical Recipes, Cambridge Univ. Press, 1992, Chapter 15.

 global   iteration  func_calls

 Npnt = length(y_dat);               % number of data points
 Ncof = length(a);                   % number of coefficients 

 if nargin < 6
   da = 0.001;
 end

%JtWJ = zeros(Ncof);
%JtWdy  = zeros(Ncof,1);

 y_hat = feval(func,t,a,c);          % evaluate model using coefficients 'a'
 func_calls = func_calls + 1;

 if ( ~rem(iteration,2*Ncof) || dX2 > 0 ) 
   J = lm_FD_J(func,t,a,y_hat,da,c);            % finite difference
 else
   J = lm_Broyden_J(a_old,y_old,J,a,y_hat);     % rank-1 update
 end

 delta_y = y_dat - y_hat;            % residual error between model and data

 Chi_sq = delta_y' * ( delta_y .* weight );     % Chi-squared error criteria

 JtWJ  = J' * ( J .* ( weight * ones(1,Ncof) ) );  

 JtWdy = J' * ( weight .* delta_y );
 
% endfunction  # ------------------------------------------------------ LM_MATX
