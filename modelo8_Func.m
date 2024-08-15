function y = modelo8_Func(u)
    x = u(1:6);
    V = u(7);
    DeltaV = u(8);
    global odeFuncSimulink;
    y = odeFuncSimulink(0, [x; V; DeltaV]);
end