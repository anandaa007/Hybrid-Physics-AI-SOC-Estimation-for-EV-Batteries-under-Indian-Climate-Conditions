function ET = apply_cooling(ET, Ru_factor)
%APPLY_COOLING  Scale the cell-to-ambient thermal resistance Ru.
%   The identified Ru is that of a single cell in a ventilated climate
%   chamber. In a pack the heat path to ambient is usually worse
%   (Ru_factor > 1: tightly packed, sealed scooter pack) or better with
%   liquid cooling (Ru_factor < 1).
ET.Ru = ET.Ru * Ru_factor;
ET.A_th = [-1/(ET.Rc*ET.Cc),  1/(ET.Rc*ET.Cc);
            1/(ET.Rc*ET.Cs), -1/(ET.Rc*ET.Cs) - 1/(ET.Ru*ET.Cs)];
ET.B_th = [1/ET.Cc, 0; 0, 1/(ET.Ru*ET.Cs)];
end
