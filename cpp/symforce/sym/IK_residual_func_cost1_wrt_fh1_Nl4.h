// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl4
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl4(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 655

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (211)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = Scalar(1.0) / (fh1);
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp3 = -2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp5 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp5 +
                       Scalar(0.20999999999999999);
  const Scalar _tmp7 = -_tmp6;
  const Scalar _tmp8 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp9 = 2 * _tmp8;
  const Scalar _tmp10 = _tmp2 * _tmp9;
  const Scalar _tmp11 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp12 = _tmp11 * _tmp4;
  const Scalar _tmp13 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp14 = _tmp4 * _tmp9;
  const Scalar _tmp15 = _tmp11 * _tmp2;
  const Scalar _tmp16 = _tmp14 - _tmp15;
  const Scalar _tmp17 = -Scalar(0.010999999999999999) * _tmp16;
  const Scalar _tmp18 = -_tmp13 + _tmp17;
  const Scalar _tmp19 = _tmp18 + _tmp7;
  const Scalar _tmp20 = _tmp19 + position_vector(1, 0);
  const Scalar _tmp21 = 1 - 2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp23 = -_tmp22;
  const Scalar _tmp24 = 2 * _tmp2 * _tmp4;
  const Scalar _tmp25 = _tmp11 * _tmp8;
  const Scalar _tmp26 = _tmp24 + _tmp25;
  const Scalar _tmp27 = -Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp10 - Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp29 = _tmp27 - _tmp28;
  const Scalar _tmp30 = _tmp23 + _tmp29;
  const Scalar _tmp31 = _tmp30 + position_vector(0, 0);
  const Scalar _tmp32 = _tmp1 * fv1;
  const Scalar _tmp33 = std::asinh(_tmp32);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * _tmp33;
  const Scalar _tmp35 =
      -Scalar(0.1034955) * _tmp34 * fh1 -
      Scalar(0.86104699584133515) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp20 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp31 - 1), Scalar(2))));
  const Scalar _tmp36 = _tmp1 * _tmp35;
  const Scalar _tmp37 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp38 = Scalar(1.0) * _tmp33;
  const Scalar _tmp39 = _tmp27 + _tmp28;
  const Scalar _tmp40 = _tmp23 + _tmp39;
  const Scalar _tmp41 = Scalar(1.0) * _tmp40;
  const Scalar _tmp42 = _tmp18 + _tmp6;
  const Scalar _tmp43 = Scalar(1.0) * _tmp42;
  const Scalar _tmp44 = -_tmp43;
  const Scalar _tmp45 = _tmp13 + _tmp17;
  const Scalar _tmp46 = _tmp45 + _tmp6;
  const Scalar _tmp47 = _tmp44 + _tmp46;
  const Scalar _tmp48 = _tmp22 + _tmp29;
  const Scalar _tmp49 = _tmp45 + _tmp7;
  const Scalar _tmp50 = Scalar(1.0) / (_tmp44 + _tmp49);
  const Scalar _tmp51 = _tmp50 * (_tmp41 - _tmp48);
  const Scalar _tmp52 = _tmp47 * _tmp51;
  const Scalar _tmp53 = _tmp22 + _tmp39;
  const Scalar _tmp54 = Scalar(1.0) / (_tmp41 - _tmp52 - _tmp53);
  const Scalar _tmp55 = Scalar(1.0) * _tmp54;
  const Scalar _tmp56 = _tmp42 + position_vector(1, 0);
  const Scalar _tmp57 = _tmp56 + Scalar(-4.8333311099999996);
  const Scalar _tmp58 = _tmp40 + position_vector(0, 0);
  const Scalar _tmp59 = _tmp58 + Scalar(1.79662371);
  const Scalar _tmp60 = Scalar(1.0) / (_tmp59);
  const Scalar _tmp61 = _tmp57 * _tmp60;
  const Scalar _tmp62 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp63 =
      -Scalar(0.010999999999999999) * _tmp21 - Scalar(0.010999999999999999) * _tmp3;
  const Scalar _tmp64 = Scalar(0.20999999999999999) * _tmp24 - Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp65 = _tmp63 - _tmp64;
  const Scalar _tmp66 = _tmp62 + _tmp65;
  const Scalar _tmp67 = _tmp49 + position_vector(1, 0);
  const Scalar _tmp68 = _tmp67 + Scalar(8.3888750099999996);
  const Scalar _tmp69 = _tmp48 + position_vector(0, 0);
  const Scalar _tmp70 = _tmp69 + Scalar(-2.5202214700000001);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp66 * _tmp72;
  const Scalar _tmp74 = -_tmp62;
  const Scalar _tmp75 = _tmp63 + _tmp64;
  const Scalar _tmp76 = _tmp74 + _tmp75;
  const Scalar _tmp77 = _tmp68 * _tmp71;
  const Scalar _tmp78 = -_tmp61 * _tmp73 + _tmp76 * _tmp77;
  const Scalar _tmp79 = Scalar(1.0) / (_tmp61 * _tmp72 - _tmp77);
  const Scalar _tmp80 = Scalar(1.0) * _tmp79;
  const Scalar _tmp81 = _tmp78 * _tmp80;
  const Scalar _tmp82 = -_tmp72 * _tmp76 + _tmp73;
  const Scalar _tmp83 = _tmp51 * _tmp81 - _tmp80 * _tmp82;
  const Scalar _tmp84 = _tmp46 + position_vector(1, 0);
  const Scalar _tmp85 = _tmp84 + Scalar(-4.7752063900000001);
  const Scalar _tmp86 = _tmp53 + position_vector(0, 0);
  const Scalar _tmp87 = _tmp86 + Scalar(-2.71799795);
  const Scalar _tmp88 = std::pow(Scalar(std::pow(_tmp85, Scalar(2)) + std::pow(_tmp87, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp89 = _tmp87 * _tmp88;
  const Scalar _tmp90 = _tmp66 * _tmp89;
  const Scalar _tmp91 = _tmp85 * _tmp88;
  const Scalar _tmp92 = _tmp61 * _tmp89 - _tmp91;
  const Scalar _tmp93 = _tmp79 * _tmp92;
  const Scalar _tmp94 = _tmp62 + _tmp75;
  const Scalar _tmp95 = -_tmp61 * _tmp90 - _tmp78 * _tmp93 + _tmp91 * _tmp94;
  const Scalar _tmp96 = -_tmp51 * _tmp95 - _tmp82 * _tmp93 - _tmp89 * _tmp94 + _tmp90;
  const Scalar _tmp97 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp98 =
      std::sqrt(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp59, Scalar(2))));
  const Scalar _tmp99 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp100 = _tmp60 * _tmp98;
  const Scalar _tmp101 = _tmp100 * (_tmp40 * _tmp57 * _tmp99 - _tmp42 * _tmp59 * _tmp99);
  const Scalar _tmp102 = _tmp101 * _tmp72 - _tmp48 * _tmp77 + _tmp49 * _tmp72;
  const Scalar _tmp103 = _tmp101 * _tmp89 - _tmp102 * _tmp93 + _tmp46 * _tmp89 - _tmp53 * _tmp91;
  const Scalar _tmp104 = _tmp103 * _tmp97;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp103);
  const Scalar _tmp106 = _tmp105 * _tmp96;
  const Scalar _tmp107 = _tmp106 * (-_tmp102 * _tmp80 - _tmp104 * _tmp83);
  const Scalar _tmp108 = _tmp47 * _tmp54;
  const Scalar _tmp109 = _tmp97 * (_tmp107 + _tmp83);
  const Scalar _tmp110 = _tmp107 * _tmp108 - _tmp109 * _tmp95 - _tmp81;
  const Scalar _tmp111 = Scalar(1.0) * _tmp50;
  const Scalar _tmp112 = _tmp31 + Scalar(1.9874742000000001);
  const Scalar _tmp113 = _tmp20 + Scalar(8.3196563700000006);
  const Scalar _tmp114 =
      std::pow(Scalar(std::pow(_tmp112, Scalar(2)) + std::pow(_tmp113, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp115 = _tmp113 * _tmp114;
  const Scalar _tmp116 = Scalar(1.0) * _tmp115 * (_tmp107 * _tmp55 - _tmp110 * _tmp111);
  const Scalar _tmp117 = _tmp65 + _tmp74;
  const Scalar _tmp118 = _tmp117 * fh1;
  const Scalar _tmp119 = -_tmp115 * _tmp118 - Scalar(5.1796800000000003) * _tmp16 - _tmp19 * fv1;
  const Scalar _tmp120 = _tmp52 * _tmp55 + Scalar(1.0);
  const Scalar _tmp121 = _tmp51 * _tmp55;
  const Scalar _tmp122 = -Scalar(1.0) * _tmp111 * _tmp120 + Scalar(1.0) * _tmp121;
  const Scalar _tmp123 = _tmp112 * _tmp114;
  const Scalar _tmp124 = _tmp118 * _tmp123 + Scalar(5.1796800000000003) * _tmp26 + _tmp30 * fv1;
  const Scalar _tmp125 = _tmp47 * _tmp55;
  const Scalar _tmp126 = _tmp125 * _tmp50;
  const Scalar _tmp127 = Scalar(1.0) * _tmp126 - Scalar(1.0) * _tmp55;
  const Scalar _tmp128 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp129 = _tmp41 + _tmp43 * _tmp51;
  const Scalar _tmp130 = 0;
  const Scalar _tmp131 = _tmp129 * _tmp54;
  const Scalar _tmp132 = -_tmp130 * _tmp95 - _tmp131 * _tmp47 + _tmp44;
  const Scalar _tmp133 = _tmp61 * _tmp79;
  const Scalar _tmp134 = _tmp133 * _tmp78 + _tmp61 * _tmp66;
  const Scalar _tmp135 = _tmp133 * _tmp82 - _tmp134 * _tmp51 - _tmp66;
  const Scalar _tmp136 = _tmp106 * (-_tmp101 + _tmp102 * _tmp133 - _tmp104 * _tmp135);
  const Scalar _tmp137 = _tmp97 * (_tmp135 + _tmp136);
  const Scalar _tmp138 = _tmp108 * _tmp136 + _tmp134 - _tmp137 * _tmp95;
  const Scalar _tmp139 = Scalar(1.0) * _tmp123 * (-_tmp111 * _tmp138 + _tmp136 * _tmp55);
  const Scalar _tmp140 = Scalar(1.0) * _tmp105;
  const Scalar _tmp141 = _tmp106 * _tmp125 - _tmp140 * _tmp95;
  const Scalar _tmp142 = _tmp106 * _tmp55;
  const Scalar _tmp143 = _tmp115 * _tmp30 - _tmp123 * _tmp19;
  const Scalar _tmp144 = Scalar(1.0) * _tmp143;
  const Scalar _tmp145 = _tmp144 * (-_tmp111 * _tmp141 + _tmp142);
  const Scalar _tmp146 =
      _tmp116 * fh1 + _tmp119 * _tmp122 + _tmp124 * _tmp127 +
      Scalar(1.0) * _tmp128 * (-_tmp111 * _tmp132 - _tmp129 * _tmp55 + Scalar(1.0)) +
      _tmp139 * fh1 + _tmp145 * fh1;
  const Scalar _tmp147 = _tmp79 * (-_tmp109 * _tmp92 + Scalar(1.0));
  const Scalar _tmp148 = _tmp100 * _tmp115 * (_tmp109 * _tmp89 + _tmp147 * _tmp72);
  const Scalar _tmp149 = _tmp105 * _tmp80 * _tmp92;
  const Scalar _tmp150 = _tmp100 * _tmp143 * (_tmp140 * _tmp89 - _tmp149 * _tmp72);
  const Scalar _tmp151 = _tmp79 * (-_tmp137 * _tmp92 - _tmp61);
  const Scalar _tmp152 = _tmp100 * _tmp123 * (_tmp137 * _tmp89 + _tmp151 * _tmp72 + Scalar(1.0));
  const Scalar _tmp153 = -_tmp100 * _tmp128 * (-_tmp130 * _tmp72 * _tmp93 + _tmp130 * _tmp89) -
                         _tmp148 * fh1 - _tmp150 * fh1 - _tmp152 * fh1;
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 = std::asinh(_tmp146 * _tmp154);
  const Scalar _tmp156 = Scalar(1.0) * _tmp155;
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp153;
  const Scalar _tmp158 =
      -_tmp155 * _tmp157 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp56), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp58 - 1), Scalar(2))));
  const Scalar _tmp159 = Scalar(0.1034955) * _tmp154;
  const Scalar _tmp160 = _tmp158 * _tmp159;
  const Scalar _tmp161 = -_tmp148 - _tmp150 - _tmp152;
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp161;
  const Scalar _tmp163 = std::pow(_tmp153, Scalar(-2));
  const Scalar _tmp164 = _tmp161 * _tmp163;
  const Scalar _tmp165 = _tmp115 * _tmp117;
  const Scalar _tmp166 = _tmp117 * _tmp123;
  const Scalar _tmp167 = (-_tmp146 * _tmp164 + _tmp154 * (_tmp116 - _tmp122 * _tmp165 +
                                                          _tmp127 * _tmp166 + _tmp139 + _tmp145)) /
                         std::sqrt(Scalar(std::pow(_tmp146, Scalar(2)) * _tmp163 + 1));
  const Scalar _tmp168 = _tmp143 * _tmp149;
  const Scalar _tmp169 = _tmp115 * _tmp147;
  const Scalar _tmp170 = _tmp128 * _tmp130;
  const Scalar _tmp171 = _tmp123 * _tmp151;
  const Scalar _tmp172 = -_tmp168 * fh1 + _tmp169 * fh1 - _tmp170 * _tmp93 + _tmp171 * fh1;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = _tmp110 * _tmp115 * _tmp50;
  const Scalar _tmp175 = _tmp141 * _tmp143 * _tmp50;
  const Scalar _tmp176 = _tmp124 * _tmp55;
  const Scalar _tmp177 = _tmp123 * _tmp138 * _tmp50;
  const Scalar _tmp178 = _tmp120 * _tmp50;
  const Scalar _tmp179 = _tmp119 * _tmp178 + _tmp128 * _tmp132 * _tmp50 + _tmp174 * fh1 +
                         _tmp175 * fh1 - _tmp176 * _tmp47 * _tmp50 + _tmp177 * fh1;
  const Scalar _tmp180 = std::asinh(_tmp173 * _tmp179);
  const Scalar _tmp181 = Scalar(9.6622558468725703) * _tmp180;
  const Scalar _tmp182 =
      -_tmp172 * _tmp181 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp69), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp67 - 1), Scalar(2))));
  const Scalar _tmp183 = Scalar(0.1034955) * _tmp173;
  const Scalar _tmp184 = _tmp182 * _tmp183;
  const Scalar _tmp185 = Scalar(1.0) * _tmp180;
  const Scalar _tmp186 = -_tmp168 + _tmp169 + _tmp171;
  const Scalar _tmp187 = std::pow(_tmp172, Scalar(-2));
  const Scalar _tmp188 = _tmp186 * _tmp187;
  const Scalar _tmp189 = Scalar(9.6622558468725703) * _tmp172;
  const Scalar _tmp190 =
      (_tmp173 * (-_tmp126 * _tmp166 - _tmp165 * _tmp178 + _tmp174 + _tmp175 + _tmp177) -
       _tmp179 * _tmp188) /
      std::sqrt(Scalar(std::pow(_tmp179, Scalar(2)) * _tmp187 + 1));
  const Scalar _tmp191 = _tmp105 * _tmp144;
  const Scalar _tmp192 = _tmp109 * _tmp115;
  const Scalar _tmp193 = _tmp123 * _tmp137;
  const Scalar _tmp194 = _tmp191 + _tmp192 + _tmp193;
  const Scalar _tmp195 = _tmp170 + _tmp191 * fh1 + _tmp192 * fh1 + _tmp193 * fh1;
  const Scalar _tmp196 = Scalar(1.0) / (_tmp195);
  const Scalar _tmp197 = _tmp142 * _tmp143;
  const Scalar _tmp198 = _tmp107 * _tmp115 * _tmp54;
  const Scalar _tmp199 = _tmp123 * _tmp136 * _tmp54;
  const Scalar _tmp200 = -_tmp119 * _tmp121 + _tmp128 * _tmp131 + _tmp176 - _tmp197 * fh1 -
                         _tmp198 * fh1 - _tmp199 * fh1;
  const Scalar _tmp201 = std::asinh(_tmp196 * _tmp200);
  const Scalar _tmp202 = Scalar(9.6622558468725703) * _tmp201;
  const Scalar _tmp203 =
      -_tmp195 * _tmp202 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp84), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp86), Scalar(2))));
  const Scalar _tmp204 = Scalar(0.1034955) * _tmp196;
  const Scalar _tmp205 = _tmp203 * _tmp204;
  const Scalar _tmp206 = Scalar(1.0) * _tmp201;
  const Scalar _tmp207 = std::pow(_tmp195, Scalar(-2));
  const Scalar _tmp208 = _tmp194 * _tmp207;
  const Scalar _tmp209 =
      (_tmp196 * (_tmp121 * _tmp165 + _tmp166 * _tmp55 - _tmp197 - _tmp198 - _tmp199) -
       _tmp200 * _tmp208) /
      std::sqrt(Scalar(std::pow(_tmp200, Scalar(2)) * _tmp207 + 1));
  const Scalar _tmp210 = Scalar(9.6622558468725703) * _tmp195;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -Scalar(8.4718465800000011) * _tmp1 -
      Scalar(9.6622558468725703) * fh1 *
          (-Scalar(1.0) * _tmp0 * _tmp37 * fv1 * std::sinh(_tmp38) -
           Scalar(0.87679799772039002) * _tmp0 -
           (-_tmp0 * _tmp35 +
            Scalar(0.1034955) * _tmp1 * (Scalar(9.6622558468725703) * _tmp32 * _tmp37 - _tmp34)) *
               std::sinh(_tmp36)) +
      Scalar(9.6622558468725703) * std::cosh(_tmp36) -
      Scalar(9.6622558468725703) * std::cosh(_tmp38);
  _res(1, 0) =
      -_tmp157 *
          (-Scalar(0.86625939559540499) * _tmp164 + Scalar(1.0) * _tmp167 * std::sinh(_tmp156) -
           (-Scalar(0.1034955) * _tmp158 * _tmp164 +
            _tmp159 * (-_tmp155 * _tmp162 - _tmp157 * _tmp167)) *
               std::sinh(_tmp160)) -
      _tmp162 * (Scalar(0.86625939559540499) * _tmp154 + std::cosh(_tmp156) - std::cosh(_tmp160));
  _res(2, 0) =
      -Scalar(9.6622558468725703) * _tmp186 *
          (Scalar(0.87653584775870996) * _tmp173 - std::cosh(_tmp184) + std::cosh(_tmp185)) -
      _tmp189 *
          (-Scalar(0.87653584775870996) * _tmp188 + Scalar(1.0) * _tmp190 * std::sinh(_tmp185) -
           (-Scalar(0.1034955) * _tmp182 * _tmp188 +
            _tmp183 * (-_tmp181 * _tmp186 - _tmp189 * _tmp190)) *
               std::sinh(_tmp184));
  _res(3, 0) =
      -Scalar(9.6622558468725703) * _tmp194 *
          (Scalar(0.86565325453551001) * _tmp196 - std::cosh(_tmp205) + std::cosh(_tmp206)) -
      _tmp210 *
          (-Scalar(0.86565325453551001) * _tmp208 + Scalar(1.0) * _tmp209 * std::sinh(_tmp206) -
           (-Scalar(0.1034955) * _tmp203 * _tmp208 +
            _tmp204 * (-_tmp194 * _tmp202 - _tmp209 * _tmp210)) *
               std::sinh(_tmp205));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
