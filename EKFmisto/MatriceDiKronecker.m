function H = MatriceDiKronecker(q, i)

    % Costruzione della matrice J
    J = [-1, zeros(1, q)];
    J(2:end) = (1:q == i); % Delta di Kronecker: 1 solo in posizione i

    % Matrice identità 2x2
    I = eye(2);

    % Prodotto di Kronecker
    H = kron(J, I);

end