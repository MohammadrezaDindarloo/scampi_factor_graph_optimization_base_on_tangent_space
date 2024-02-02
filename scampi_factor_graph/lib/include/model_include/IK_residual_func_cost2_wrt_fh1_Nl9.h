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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl9
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl9(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 640

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (211)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = _tmp0 * fv1;
  const Scalar _tmp2 = std::asinh(_tmp1);
  const Scalar _tmp3 = Scalar(1.0) * _tmp2;
  const Scalar _tmp4 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp5 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp6 = -2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp8 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp9 = Scalar(0.20999999999999999) * _tmp6 + Scalar(0.20999999999999999) * _tmp8 +
                       Scalar(0.20999999999999999);
  const Scalar _tmp10 = -_tmp9;
  const Scalar _tmp11 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                        _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp12 = 2 * _tmp11 * _tmp7;
  const Scalar _tmp13 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp14 = _tmp13 * _tmp5;
  const Scalar _tmp15 = _tmp12 - _tmp14;
  const Scalar _tmp16 = -Scalar(0.010999999999999999) * _tmp15;
  const Scalar _tmp17 = 2 * _tmp5;
  const Scalar _tmp18 = _tmp11 * _tmp17;
  const Scalar _tmp19 = _tmp13 * _tmp7;
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp18 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp21 = _tmp16 + _tmp20;
  const Scalar _tmp22 = _tmp10 + _tmp21;
  const Scalar _tmp23 = _tmp22 + position_vector(1, 0);
  const Scalar _tmp24 = 1 - 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp26 = _tmp17 * _tmp7;
  const Scalar _tmp27 = _tmp11 * _tmp13;
  const Scalar _tmp28 = _tmp26 + _tmp27;
  const Scalar _tmp29 = -Scalar(0.010999999999999999) * _tmp28;
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp18 - Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp31 = _tmp29 - _tmp30;
  const Scalar _tmp32 = _tmp25 + _tmp31;
  const Scalar _tmp33 = _tmp32 + position_vector(0, 0);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * _tmp2;
  const Scalar _tmp35 =
      -Scalar(0.1034955) * _tmp34 * fh1 -
      Scalar(0.868210813597455) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp33), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp23 - 1), Scalar(2))));
  const Scalar _tmp36 =
      std::pow(Scalar(_tmp4 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp37 = _tmp0 * _tmp35;
  const Scalar _tmp38 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp39 = -_tmp25;
  const Scalar _tmp40 = _tmp31 + _tmp39;
  const Scalar _tmp41 = _tmp40 + position_vector(0, 0);
  const Scalar _tmp42 = _tmp41 + Scalar(1.9874742000000001);
  const Scalar _tmp43 = _tmp16 - _tmp20;
  const Scalar _tmp44 = _tmp10 + _tmp43;
  const Scalar _tmp45 = _tmp44 + position_vector(1, 0);
  const Scalar _tmp46 = _tmp45 + Scalar(8.3196563700000006);
  const Scalar _tmp47 = std::pow(Scalar(std::pow(_tmp42, Scalar(2)) + std::pow(_tmp46, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp48 = _tmp42 * _tmp47;
  const Scalar _tmp49 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp50 =
      -Scalar(0.010999999999999999) * _tmp24 - Scalar(0.010999999999999999) * _tmp6;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp26 - Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp52 = _tmp50 - _tmp51;
  const Scalar _tmp53 = _tmp49 + _tmp52;
  const Scalar _tmp54 = _tmp29 + _tmp30;
  const Scalar _tmp55 = _tmp39 + _tmp54;
  const Scalar _tmp56 = _tmp55 + position_vector(0, 0);
  const Scalar _tmp57 = _tmp56 + Scalar(1.79662371);
  const Scalar _tmp58 = _tmp43 + _tmp9;
  const Scalar _tmp59 = _tmp58 + position_vector(1, 0);
  const Scalar _tmp60 = _tmp59 + Scalar(-4.8333311099999996);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp57 * _tmp61;
  const Scalar _tmp63 = _tmp50 + _tmp51;
  const Scalar _tmp64 = _tmp49 + _tmp63;
  const Scalar _tmp65 = -_tmp53 * _tmp62 + _tmp62 * _tmp64;
  const Scalar _tmp66 = _tmp46 * _tmp47;
  const Scalar _tmp67 = _tmp21 + _tmp9;
  const Scalar _tmp68 = _tmp67 + position_vector(1, 0);
  const Scalar _tmp69 = _tmp68 + Scalar(-4.7752063900000001);
  const Scalar _tmp70 = _tmp25 + _tmp54;
  const Scalar _tmp71 = _tmp70 + position_vector(0, 0);
  const Scalar _tmp72 = _tmp71 + Scalar(-2.71799795);
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 = _tmp69 * _tmp73;
  const Scalar _tmp75 = _tmp48 * _tmp74 - _tmp66;
  const Scalar _tmp76 = _tmp60 * _tmp61;
  const Scalar _tmp77 = Scalar(1.0) / (_tmp62 * _tmp74 - _tmp76);
  const Scalar _tmp78 = _tmp75 * _tmp77;
  const Scalar _tmp79 = -_tmp49;
  const Scalar _tmp80 = _tmp52 + _tmp79;
  const Scalar _tmp81 = _tmp64 * _tmp74;
  const Scalar _tmp82 = _tmp53 * _tmp76 - _tmp62 * _tmp81;
  const Scalar _tmp83 = -_tmp48 * _tmp81 + _tmp66 * _tmp80 - _tmp78 * _tmp82;
  const Scalar _tmp84 = Scalar(1.0) * _tmp67;
  const Scalar _tmp85 = -_tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp58 + _tmp85);
  const Scalar _tmp87 = Scalar(1.0) * _tmp70;
  const Scalar _tmp88 = _tmp86 * (-_tmp55 + _tmp87);
  const Scalar _tmp89 = _tmp48 * _tmp64 - _tmp48 * _tmp80 - _tmp65 * _tmp78 - _tmp83 * _tmp88;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp89);
  const Scalar _tmp91 = _tmp84 * _tmp88 + _tmp87;
  const Scalar _tmp92 = 0;
  const Scalar _tmp93 = _tmp62 * _tmp78;
  const Scalar _tmp94 =
      std::sqrt(Scalar(std::pow(_tmp69, Scalar(2)) + std::pow(_tmp72, Scalar(2))));
  const Scalar _tmp95 = _tmp73 * _tmp94;
  const Scalar _tmp96 = _tmp23 + Scalar(8.3888750099999996);
  const Scalar _tmp97 = _tmp33 + Scalar(-2.5202214700000001);
  const Scalar _tmp98 = std::pow(Scalar(std::pow(_tmp96, Scalar(2)) + std::pow(_tmp97, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp99 = _tmp96 * _tmp98;
  const Scalar _tmp100 = _tmp97 * _tmp98;
  const Scalar _tmp101 = -_tmp100 * _tmp22 + _tmp32 * _tmp99;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp103 = _tmp95 * (-_tmp102 * _tmp67 * _tmp72 + _tmp102 * _tmp69 * _tmp70);
  const Scalar _tmp104 = _tmp103 * _tmp62 - _tmp55 * _tmp76 + _tmp58 * _tmp62;
  const Scalar _tmp105 = _tmp103 * _tmp48 - _tmp104 * _tmp78 - _tmp40 * _tmp66 + _tmp44 * _tmp48;
  const Scalar _tmp106 = Scalar(1.0) / (_tmp105);
  const Scalar _tmp107 = Scalar(1.0) * _tmp106;
  const Scalar _tmp108 = _tmp101 * _tmp95 * (_tmp107 * _tmp48 - _tmp107 * _tmp93);
  const Scalar _tmp109 = Scalar(1.0) * _tmp77;
  const Scalar _tmp110 = _tmp109 * _tmp82;
  const Scalar _tmp111 = -_tmp109 * _tmp65 + _tmp110 * _tmp88;
  const Scalar _tmp112 = _tmp105 * _tmp90;
  const Scalar _tmp113 = _tmp106 * _tmp89;
  const Scalar _tmp114 = _tmp113 * (-_tmp104 * _tmp109 - _tmp111 * _tmp112);
  const Scalar _tmp115 = _tmp90 * (_tmp111 + _tmp114);
  const Scalar _tmp116 = -_tmp115 * _tmp75 + Scalar(1.0);
  const Scalar _tmp117 = _tmp62 * _tmp77;
  const Scalar _tmp118 = _tmp95 * _tmp99 * (_tmp115 * _tmp48 + _tmp116 * _tmp117);
  const Scalar _tmp119 = _tmp74 * _tmp77;
  const Scalar _tmp120 = _tmp119 * _tmp82 + _tmp81;
  const Scalar _tmp121 = _tmp119 * _tmp65 - _tmp120 * _tmp88 - _tmp64;
  const Scalar _tmp122 = _tmp113 * (-_tmp103 + _tmp104 * _tmp119 - _tmp112 * _tmp121);
  const Scalar _tmp123 = _tmp90 * (_tmp121 + _tmp122);
  const Scalar _tmp124 = -_tmp123 * _tmp75 - _tmp74;
  const Scalar _tmp125 = _tmp100 * _tmp95 * (_tmp117 * _tmp124 + _tmp123 * _tmp48 + Scalar(1.0));
  const Scalar _tmp126 = -_tmp108 * fh1 - _tmp118 * fh1 - _tmp125 * fh1 -
                         _tmp38 * _tmp95 * (_tmp48 * _tmp92 - _tmp92 * _tmp93);
  const Scalar _tmp127 = Scalar(1.0) / (_tmp126);
  const Scalar _tmp128 = _tmp44 + _tmp85;
  const Scalar _tmp129 = _tmp128 * _tmp88;
  const Scalar _tmp130 = Scalar(1.0) / (-_tmp129 - _tmp40 + _tmp87);
  const Scalar _tmp131 = _tmp128 * _tmp130;
  const Scalar _tmp132 = -_tmp110 + _tmp114 * _tmp131 - _tmp115 * _tmp83;
  const Scalar _tmp133 = Scalar(1.0) * _tmp86;
  const Scalar _tmp134 = Scalar(1.0) * _tmp130;
  const Scalar _tmp135 = Scalar(1.0) * _tmp99 * (_tmp114 * _tmp134 - _tmp132 * _tmp133);
  const Scalar _tmp136 = _tmp63 + _tmp79;
  const Scalar _tmp137 = _tmp136 * fh1;
  const Scalar _tmp138 = -_tmp137 * _tmp99 - Scalar(5.1796800000000003) * _tmp15 - _tmp22 * fv1;
  const Scalar _tmp139 = _tmp129 * _tmp134 + Scalar(1.0);
  const Scalar _tmp140 = _tmp134 * _tmp88;
  const Scalar _tmp141 = -Scalar(1.0) * _tmp133 * _tmp139 + Scalar(1.0) * _tmp140;
  const Scalar _tmp142 = _tmp130 * _tmp91;
  const Scalar _tmp143 = -_tmp128 * _tmp142 - _tmp83 * _tmp92 + _tmp85;
  const Scalar _tmp144 = _tmp120 + _tmp122 * _tmp131 - _tmp123 * _tmp83;
  const Scalar _tmp145 = Scalar(1.0) * _tmp100 * (_tmp122 * _tmp134 - _tmp133 * _tmp144);
  const Scalar _tmp146 = _tmp100 * _tmp137 + Scalar(5.1796800000000003) * _tmp28 + _tmp32 * fv1;
  const Scalar _tmp147 = _tmp128 * _tmp86;
  const Scalar _tmp148 = Scalar(1.0) * _tmp134 * _tmp147 - Scalar(1.0) * _tmp134;
  const Scalar _tmp149 = _tmp113 * _tmp134;
  const Scalar _tmp150 = _tmp86 * (-_tmp107 * _tmp83 + _tmp128 * _tmp149);
  const Scalar _tmp151 = Scalar(1.0) * _tmp101;
  const Scalar _tmp152 = _tmp151 * (_tmp149 - Scalar(1.0) * _tmp150);
  const Scalar _tmp153 =
      _tmp135 * fh1 + _tmp138 * _tmp141 + _tmp145 * fh1 + _tmp146 * _tmp148 + _tmp152 * fh1 +
      Scalar(1.0) * _tmp38 * (-_tmp133 * _tmp143 - _tmp134 * _tmp91 + Scalar(1.0));
  const Scalar _tmp154 = std::asinh(_tmp127 * _tmp153);
  const Scalar _tmp155 = Scalar(1.0) * _tmp154;
  const Scalar _tmp156 = std::pow(_tmp126, Scalar(-2));
  const Scalar _tmp157 = _tmp100 * _tmp136;
  const Scalar _tmp158 = _tmp136 * _tmp99;
  const Scalar _tmp159 = -_tmp108 - _tmp118 - _tmp125;
  const Scalar _tmp160 = _tmp156 * _tmp159;
  const Scalar _tmp161 =
      (_tmp127 * (_tmp135 - _tmp141 * _tmp158 + _tmp145 + _tmp148 * _tmp157 + _tmp152) -
       _tmp153 * _tmp160) /
      std::sqrt(Scalar(std::pow(_tmp153, Scalar(2)) * _tmp156 + 1));
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp154;
  const Scalar _tmp163 =
      -_tmp126 * _tmp162 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp68), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp71), Scalar(2))));
  const Scalar _tmp164 = Scalar(0.1034955) * _tmp127;
  const Scalar _tmp165 = _tmp163 * _tmp164;
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp126;
  const Scalar _tmp167 = _tmp38 * _tmp92;
  const Scalar _tmp168 = _tmp116 * _tmp77 * _tmp99;
  const Scalar _tmp169 = _tmp100 * _tmp124 * _tmp77;
  const Scalar _tmp170 = _tmp106 * _tmp151;
  const Scalar _tmp171 = _tmp170 * fh1;
  const Scalar _tmp172 = -_tmp167 * _tmp78 + _tmp168 * fh1 + _tmp169 * fh1 - _tmp171 * _tmp78;
  const Scalar _tmp173 = Scalar(1.0) / (_tmp172);
  const Scalar _tmp174 = _tmp134 * _tmp146;
  const Scalar _tmp175 = _tmp139 * _tmp86;
  const Scalar _tmp176 = _tmp132 * _tmp86 * _tmp99;
  const Scalar _tmp177 = _tmp100 * _tmp86;
  const Scalar _tmp178 = _tmp144 * _tmp177;
  const Scalar _tmp179 = _tmp101 * _tmp150;
  const Scalar _tmp180 = _tmp138 * _tmp175 + _tmp143 * _tmp38 * _tmp86 - _tmp147 * _tmp174 +
                         _tmp176 * fh1 + _tmp178 * fh1 + _tmp179 * fh1;
  const Scalar _tmp181 = std::asinh(_tmp173 * _tmp180);
  const Scalar _tmp182 = Scalar(9.6622558468725703) * _tmp172;
  const Scalar _tmp183 =
      -_tmp181 * _tmp182 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp59), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp56 - 1), Scalar(2))));
  const Scalar _tmp184 = Scalar(0.1034955) * _tmp173;
  const Scalar _tmp185 = _tmp183 * _tmp184;
  const Scalar _tmp186 = Scalar(1.0) * _tmp181;
  const Scalar _tmp187 = _tmp168 + _tmp169 - _tmp170 * _tmp78;
  const Scalar _tmp188 = Scalar(9.6622558468725703) * _tmp187;
  const Scalar _tmp189 = std::pow(_tmp172, Scalar(-2));
  const Scalar _tmp190 = _tmp187 * _tmp189;
  const Scalar _tmp191 = (_tmp173 * (-_tmp128 * _tmp134 * _tmp136 * _tmp177 - _tmp158 * _tmp175 +
                                     _tmp176 + _tmp178 + _tmp179) -
                          _tmp180 * _tmp190) /
                         std::sqrt(Scalar(std::pow(_tmp180, Scalar(2)) * _tmp189 + 1));
  const Scalar _tmp192 = _tmp101 * _tmp149;
  const Scalar _tmp193 = _tmp114 * _tmp130 * _tmp99;
  const Scalar _tmp194 = _tmp100 * _tmp122 * _tmp130;
  const Scalar _tmp195 = -_tmp138 * _tmp140 + _tmp142 * _tmp38 + _tmp174 - _tmp192 * fh1 -
                         _tmp193 * fh1 - _tmp194 * fh1;
  const Scalar _tmp196 = _tmp115 * _tmp99;
  const Scalar _tmp197 = _tmp100 * _tmp123;
  const Scalar _tmp198 = _tmp167 + _tmp171 + _tmp196 * fh1 + _tmp197 * fh1;
  const Scalar _tmp199 = Scalar(1.0) / (_tmp198);
  const Scalar _tmp200 = std::asinh(_tmp195 * _tmp199);
  const Scalar _tmp201 = Scalar(9.6622558468725703) * _tmp198;
  const Scalar _tmp202 =
      -_tmp200 * _tmp201 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp41 - 1), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp45 - 1), Scalar(2))));
  const Scalar _tmp203 = Scalar(0.1034955) * _tmp199;
  const Scalar _tmp204 = _tmp202 * _tmp203;
  const Scalar _tmp205 = Scalar(1.0) * _tmp200;
  const Scalar _tmp206 = _tmp170 + _tmp196 + _tmp197;
  const Scalar _tmp207 = Scalar(9.6622558468725703) * _tmp206;
  const Scalar _tmp208 = std::pow(_tmp198, Scalar(-2));
  const Scalar _tmp209 = _tmp206 * _tmp208;
  const Scalar _tmp210 = (-_tmp195 * _tmp209 + _tmp199 * (_tmp134 * _tmp157 + _tmp140 * _tmp158 -
                                                          _tmp192 - _tmp193 - _tmp194)) /
                         std::sqrt(Scalar(std::pow(_tmp195, Scalar(2)) * _tmp208 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      Scalar(9.6622558468725703) * fh1 *
          (Scalar(1.0) * _tmp36 * _tmp4 * fv1 * std::cosh(_tmp3) -
           (Scalar(0.1034955) * _tmp0 * (Scalar(9.6622558468725703) * _tmp1 * _tmp36 - _tmp34) -
            _tmp35 * _tmp4) *
               std::cosh(_tmp37)) -
      Scalar(9.6622558468725703) * std::sinh(_tmp3) -
      Scalar(9.6622558468725703) * std::sinh(_tmp37);
  _res(1, 0) = Scalar(9.6622558468725703) * _tmp159 * (-std::sinh(_tmp155) - std::sinh(_tmp165)) +
               _tmp166 * (-Scalar(1.0) * _tmp161 * std::cosh(_tmp155) -
                          (-Scalar(0.1034955) * _tmp160 * _tmp163 +
                           _tmp164 * (-_tmp159 * _tmp162 - _tmp161 * _tmp166)) *
                              std::cosh(_tmp165));
  _res(2, 0) = _tmp182 * (-Scalar(1.0) * _tmp191 * std::cosh(_tmp186) -
                          (-Scalar(0.1034955) * _tmp183 * _tmp190 +
                           _tmp184 * (-_tmp181 * _tmp188 - _tmp182 * _tmp191)) *
                              std::cosh(_tmp185)) +
               _tmp188 * (-std::sinh(_tmp185) - std::sinh(_tmp186));
  _res(3, 0) = _tmp201 * (-Scalar(1.0) * _tmp210 * std::cosh(_tmp205) -
                          (-Scalar(0.1034955) * _tmp202 * _tmp209 +
                           _tmp203 * (-_tmp200 * _tmp207 - _tmp201 * _tmp210)) *
                              std::cosh(_tmp204)) +
               _tmp207 * (-std::sinh(_tmp204) - std::sinh(_tmp205));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
