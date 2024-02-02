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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl22
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl22(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 653

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (209)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = Scalar(1.0) / (fh1);
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp4 = 2 * _tmp3;
  const Scalar _tmp5 = _tmp2 * _tmp4;
  const Scalar _tmp6 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp7 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp8 = _tmp6 * _tmp7;
  const Scalar _tmp9 = Scalar(0.20999999999999999) * _tmp5 - Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp10 = 2 * _tmp2 * _tmp6;
  const Scalar _tmp11 = _tmp3 * _tmp7;
  const Scalar _tmp12 = _tmp10 + _tmp11;
  const Scalar _tmp13 = -Scalar(0.010999999999999999) * _tmp12;
  const Scalar _tmp14 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp15 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp14 +
                        Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999);
  const Scalar _tmp17 = _tmp13 - _tmp16;
  const Scalar _tmp18 = _tmp17 + _tmp9;
  const Scalar _tmp19 = _tmp18 + position_vector(0, 0);
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp5 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp21 = -_tmp20;
  const Scalar _tmp22 = 1 - 2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp24 = _tmp4 * _tmp6;
  const Scalar _tmp25 = _tmp2 * _tmp7;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = -Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp23 + _tmp27;
  const Scalar _tmp29 = _tmp21 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(1, 0);
  const Scalar _tmp31 = _tmp1 * fv1;
  const Scalar _tmp32 = std::asinh(_tmp31);
  const Scalar _tmp33 = Scalar(9.6622558468725703) * _tmp32;
  const Scalar _tmp34 =
      -Scalar(0.1034955) * _tmp33 * fh1 -
      Scalar(0.50022801989500498) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp30), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp19 - 1), Scalar(2))));
  const Scalar _tmp35 = _tmp1 * _tmp34;
  const Scalar _tmp36 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp37 = Scalar(1.0) * _tmp32;
  const Scalar _tmp38 = -_tmp9;
  const Scalar _tmp39 = _tmp17 + _tmp38;
  const Scalar _tmp40 = _tmp39 + position_vector(0, 0);
  const Scalar _tmp41 = _tmp40 + Scalar(1.9874742000000001);
  const Scalar _tmp42 = -_tmp23 + _tmp27;
  const Scalar _tmp43 = _tmp21 + _tmp42;
  const Scalar _tmp44 = _tmp43 + position_vector(1, 0);
  const Scalar _tmp45 = _tmp44 + Scalar(8.3196563700000006);
  const Scalar _tmp46 = std::pow(Scalar(std::pow(_tmp41, Scalar(2)) + std::pow(_tmp45, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp47 = _tmp41 * _tmp46;
  const Scalar _tmp48 = _tmp45 * _tmp46;
  const Scalar _tmp49 = _tmp20 + _tmp28;
  const Scalar _tmp50 = _tmp49 + position_vector(1, 0);
  const Scalar _tmp51 = _tmp50 + Scalar(-4.7752063900000001);
  const Scalar _tmp52 = _tmp13 + _tmp16;
  const Scalar _tmp53 = _tmp52 + _tmp9;
  const Scalar _tmp54 = _tmp53 + position_vector(0, 0);
  const Scalar _tmp55 = _tmp54 + Scalar(-2.71799795);
  const Scalar _tmp56 = Scalar(1.0) / (_tmp55);
  const Scalar _tmp57 = _tmp51 * _tmp56;
  const Scalar _tmp58 = Scalar(1.0) / (_tmp47 * _tmp57 - _tmp48);
  const Scalar _tmp59 = _tmp20 + _tmp42;
  const Scalar _tmp60 = _tmp59 + position_vector(1, 0);
  const Scalar _tmp61 = _tmp60 + Scalar(8.3888750099999996);
  const Scalar _tmp62 = _tmp38 + _tmp52;
  const Scalar _tmp63 = _tmp62 + position_vector(0, 0);
  const Scalar _tmp64 = _tmp63 + Scalar(-2.5202214700000001);
  const Scalar _tmp65 = std::pow(Scalar(std::pow(_tmp61, Scalar(2)) + std::pow(_tmp64, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp66 = _tmp61 * _tmp65;
  const Scalar _tmp67 = _tmp64 * _tmp65;
  const Scalar _tmp68 = _tmp57 * _tmp67 - _tmp66;
  const Scalar _tmp69 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp70 =
      -Scalar(0.010999999999999999) * _tmp14 - Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp71 = Scalar(0.20999999999999999) * _tmp10 - Scalar(0.20999999999999999) * _tmp11;
  const Scalar _tmp72 = _tmp70 + _tmp71;
  const Scalar _tmp73 = _tmp69 + _tmp72;
  const Scalar _tmp74 = _tmp57 * _tmp73;
  const Scalar _tmp75 = -_tmp69;
  const Scalar _tmp76 = _tmp72 + _tmp75;
  const Scalar _tmp77 = _tmp70 - _tmp71;
  const Scalar _tmp78 = _tmp75 + _tmp77;
  const Scalar _tmp79 = _tmp58 * (-_tmp47 * _tmp74 + _tmp48 * _tmp78);
  const Scalar _tmp80 = _tmp66 * _tmp76 - _tmp67 * _tmp74 - _tmp68 * _tmp79;
  const Scalar _tmp81 = Scalar(1.0) * _tmp49;
  const Scalar _tmp82 = -_tmp81;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp43 + _tmp82);
  const Scalar _tmp84 = Scalar(1.0) * _tmp53;
  const Scalar _tmp85 = -_tmp39 + _tmp84;
  const Scalar _tmp86 = _tmp83 * _tmp85;
  const Scalar _tmp87 = _tmp47 * _tmp73 - _tmp47 * _tmp78;
  const Scalar _tmp88 = _tmp58 * _tmp68;
  const Scalar _tmp89 = _tmp67 * _tmp73 - _tmp67 * _tmp76 - _tmp80 * _tmp86 - _tmp87 * _tmp88;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp89);
  const Scalar _tmp91 =
      std::sqrt(Scalar(std::pow(_tmp51, Scalar(2)) + std::pow(_tmp55, Scalar(2))));
  const Scalar _tmp92 = Scalar(1.0) / (_tmp91);
  const Scalar _tmp93 = _tmp56 * _tmp91;
  const Scalar _tmp94 = _tmp93 * (-_tmp49 * _tmp55 * _tmp92 + _tmp51 * _tmp53 * _tmp92);
  const Scalar _tmp95 = -_tmp39 * _tmp48 + _tmp43 * _tmp47 + _tmp47 * _tmp94;
  const Scalar _tmp96 = Scalar(1.0) * _tmp58;
  const Scalar _tmp97 = Scalar(1.0) * _tmp83;
  const Scalar _tmp98 = _tmp79 * _tmp85 * _tmp97 - _tmp87 * _tmp96;
  const Scalar _tmp99 = _tmp59 * _tmp67 - _tmp62 * _tmp66 + _tmp67 * _tmp94 - _tmp88 * _tmp95;
  const Scalar _tmp100 = _tmp90 * _tmp99;
  const Scalar _tmp101 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp102 = _tmp101 * _tmp89;
  const Scalar _tmp103 = _tmp102 * (-_tmp100 * _tmp98 - _tmp95 * _tmp96);
  const Scalar _tmp104 = _tmp90 * (_tmp103 + _tmp98);
  const Scalar _tmp105 = _tmp58 * (-_tmp104 * _tmp68 + Scalar(1.0));
  const Scalar _tmp106 = _tmp19 + Scalar(1.79662371);
  const Scalar _tmp107 = _tmp30 + Scalar(-4.8333311099999996);
  const Scalar _tmp108 =
      std::pow(Scalar(std::pow(_tmp106, Scalar(2)) + std::pow(_tmp107, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp109 = _tmp107 * _tmp108;
  const Scalar _tmp110 = _tmp109 * _tmp93 * (_tmp104 * _tmp67 + _tmp105 * _tmp47);
  const Scalar _tmp111 = _tmp57 * _tmp58;
  const Scalar _tmp112 = _tmp57 * _tmp79 + _tmp74;
  const Scalar _tmp113 = _tmp111 * _tmp87 - _tmp112 * _tmp86 - _tmp73;
  const Scalar _tmp114 = _tmp102 * (-_tmp100 * _tmp113 + _tmp111 * _tmp95 - _tmp94);
  const Scalar _tmp115 = _tmp90 * (_tmp113 + _tmp114);
  const Scalar _tmp116 = _tmp58 * (-_tmp115 * _tmp68 - _tmp57);
  const Scalar _tmp117 = _tmp106 * _tmp108;
  const Scalar _tmp118 = _tmp117 * _tmp93 * (_tmp115 * _tmp67 + _tmp116 * _tmp47 + Scalar(1.0));
  const Scalar _tmp119 = _tmp109 * _tmp18 - _tmp117 * _tmp29;
  const Scalar _tmp120 = Scalar(1.0) * _tmp101;
  const Scalar _tmp121 = _tmp47 * _tmp88;
  const Scalar _tmp122 = _tmp119 * _tmp93 * (-_tmp120 * _tmp121 + _tmp120 * _tmp67);
  const Scalar _tmp123 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp124 = _tmp81 * _tmp86 + _tmp84;
  const Scalar _tmp125 = 0;
  const Scalar _tmp126 = -_tmp110 * fh1 - _tmp118 * fh1 - _tmp122 * fh1 -
                         _tmp123 * _tmp93 * (-_tmp121 * _tmp125 + _tmp125 * _tmp67);
  const Scalar _tmp127 = std::pow(_tmp126, Scalar(-2));
  const Scalar _tmp128 = -_tmp110 - _tmp118 - _tmp122;
  const Scalar _tmp129 = _tmp127 * _tmp128;
  const Scalar _tmp130 = _tmp59 + _tmp82;
  const Scalar _tmp131 = _tmp130 * _tmp86;
  const Scalar _tmp132 = Scalar(1.0) / (-_tmp131 - _tmp62 + _tmp84);
  const Scalar _tmp133 = Scalar(1.0) * _tmp132;
  const Scalar _tmp134 = _tmp130 * _tmp132;
  const Scalar _tmp135 = _tmp112 + _tmp114 * _tmp134 - _tmp115 * _tmp80;
  const Scalar _tmp136 = Scalar(1.0) * _tmp117 * (_tmp114 * _tmp133 - _tmp135 * _tmp97);
  const Scalar _tmp137 = _tmp103 * _tmp134 - _tmp104 * _tmp80 - Scalar(1.0) * _tmp79;
  const Scalar _tmp138 = Scalar(1.0) * _tmp109 * (_tmp103 * _tmp133 - _tmp137 * _tmp97);
  const Scalar _tmp139 = _tmp102 * _tmp133;
  const Scalar _tmp140 = _tmp130 * _tmp133;
  const Scalar _tmp141 = _tmp102 * _tmp140 - _tmp120 * _tmp80;
  const Scalar _tmp142 = Scalar(1.0) * _tmp119;
  const Scalar _tmp143 = _tmp142 * (_tmp139 - _tmp141 * _tmp97);
  const Scalar _tmp144 = _tmp69 + _tmp77;
  const Scalar _tmp145 = _tmp144 * fh1;
  const Scalar _tmp146 = _tmp117 * _tmp145 + Scalar(5.1796800000000003) * _tmp12 + _tmp18 * fv1;
  const Scalar _tmp147 = -Scalar(1.0) * _tmp133 + Scalar(1.0) * _tmp140 * _tmp83;
  const Scalar _tmp148 = _tmp124 * _tmp132;
  const Scalar _tmp149 = -_tmp125 * _tmp80 - _tmp130 * _tmp148 + _tmp82;
  const Scalar _tmp150 = -_tmp109 * _tmp145 - Scalar(5.1796800000000003) * _tmp26 - _tmp29 * fv1;
  const Scalar _tmp151 = _tmp83 * (_tmp131 * _tmp133 + Scalar(1.0));
  const Scalar _tmp152 = _tmp133 * _tmp86;
  const Scalar _tmp153 = -Scalar(1.0) * _tmp151 + Scalar(1.0) * _tmp152;
  const Scalar _tmp154 =
      Scalar(1.0) * _tmp123 * (-_tmp124 * _tmp133 - _tmp149 * _tmp97 + Scalar(1.0)) +
      _tmp136 * fh1 + _tmp138 * fh1 + _tmp143 * fh1 + _tmp146 * _tmp147 + _tmp150 * _tmp153;
  const Scalar _tmp155 = Scalar(1.0) / (_tmp126);
  const Scalar _tmp156 = std::asinh(_tmp154 * _tmp155);
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp126;
  const Scalar _tmp158 =
      -_tmp156 * _tmp157 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp50), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp54), Scalar(2))));
  const Scalar _tmp159 = Scalar(0.1034955) * _tmp155;
  const Scalar _tmp160 = _tmp158 * _tmp159;
  const Scalar _tmp161 = _tmp109 * _tmp144;
  const Scalar _tmp162 = _tmp117 * _tmp144;
  const Scalar _tmp163 = (-_tmp129 * _tmp154 + _tmp155 * (_tmp136 + _tmp138 + _tmp143 +
                                                          _tmp147 * _tmp162 - _tmp153 * _tmp161)) /
                         std::sqrt(Scalar(_tmp127 * std::pow(_tmp154, Scalar(2)) + 1));
  const Scalar _tmp164 = Scalar(9.6622558468725703) * _tmp128;
  const Scalar _tmp165 = Scalar(1.0) * _tmp156;
  const Scalar _tmp166 = _tmp119 * _tmp141 * _tmp83;
  const Scalar _tmp167 = _tmp117 * _tmp83;
  const Scalar _tmp168 = _tmp135 * _tmp167;
  const Scalar _tmp169 = _tmp109 * _tmp137 * _tmp83;
  const Scalar _tmp170 = _tmp133 * _tmp146;
  const Scalar _tmp171 = _tmp123 * _tmp149 * _tmp83 - _tmp130 * _tmp170 * _tmp83 +
                         _tmp150 * _tmp151 + _tmp166 * fh1 + _tmp168 * fh1 + _tmp169 * fh1;
  const Scalar _tmp172 = _tmp123 * _tmp125;
  const Scalar _tmp173 = _tmp116 * _tmp117;
  const Scalar _tmp174 = _tmp101 * _tmp142;
  const Scalar _tmp175 = _tmp174 * fh1;
  const Scalar _tmp176 = _tmp105 * _tmp109;
  const Scalar _tmp177 = -_tmp172 * _tmp88 + _tmp173 * fh1 - _tmp175 * _tmp88 + _tmp176 * fh1;
  const Scalar _tmp178 = Scalar(1.0) / (_tmp177);
  const Scalar _tmp179 = std::asinh(_tmp171 * _tmp178);
  const Scalar _tmp180 = Scalar(9.6622558468725703) * _tmp177;
  const Scalar _tmp181 =
      -_tmp179 * _tmp180 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp40 - 1), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp44 - 1), Scalar(2))));
  const Scalar _tmp182 = std::pow(_tmp177, Scalar(-2));
  const Scalar _tmp183 = _tmp173 - _tmp174 * _tmp88 + _tmp176;
  const Scalar _tmp184 = _tmp182 * _tmp183;
  const Scalar _tmp185 = Scalar(9.6622558468725703) * _tmp183;
  const Scalar _tmp186 =
      (-_tmp171 * _tmp184 +
       _tmp178 * (-_tmp140 * _tmp144 * _tmp167 - _tmp151 * _tmp161 + _tmp166 + _tmp168 + _tmp169)) /
      std::sqrt(Scalar(std::pow(_tmp171, Scalar(2)) * _tmp182 + 1));
  const Scalar _tmp187 = Scalar(0.1034955) * _tmp178;
  const Scalar _tmp188 = _tmp181 * _tmp187;
  const Scalar _tmp189 = Scalar(1.0) * _tmp179;
  const Scalar _tmp190 = _tmp114 * _tmp117 * _tmp132;
  const Scalar _tmp191 = _tmp103 * _tmp109 * _tmp132;
  const Scalar _tmp192 = _tmp119 * _tmp139;
  const Scalar _tmp193 = _tmp123 * _tmp148 - _tmp150 * _tmp152 + _tmp170 - _tmp190 * fh1 -
                         _tmp191 * fh1 - _tmp192 * fh1;
  const Scalar _tmp194 = _tmp104 * _tmp109;
  const Scalar _tmp195 = _tmp115 * _tmp117;
  const Scalar _tmp196 = _tmp172 + _tmp175 + _tmp194 * fh1 + _tmp195 * fh1;
  const Scalar _tmp197 = Scalar(1.0) / (_tmp196);
  const Scalar _tmp198 = std::asinh(_tmp193 * _tmp197);
  const Scalar _tmp199 = Scalar(9.6622558468725703) * _tmp196;
  const Scalar _tmp200 =
      -_tmp198 * _tmp199 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp63), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp60 - 1), Scalar(2))));
  const Scalar _tmp201 = Scalar(0.1034955) * _tmp197;
  const Scalar _tmp202 = _tmp200 * _tmp201;
  const Scalar _tmp203 = std::pow(_tmp196, Scalar(-2));
  const Scalar _tmp204 = _tmp174 + _tmp194 + _tmp195;
  const Scalar _tmp205 = _tmp203 * _tmp204;
  const Scalar _tmp206 = Scalar(9.6622558468725703) * _tmp204;
  const Scalar _tmp207 = (-_tmp193 * _tmp205 + _tmp197 * (_tmp133 * _tmp162 + _tmp152 * _tmp161 -
                                                          _tmp190 - _tmp191 - _tmp192)) /
                         std::sqrt(Scalar(std::pow(_tmp193, Scalar(2)) * _tmp203 + 1));
  const Scalar _tmp208 = Scalar(1.0) * _tmp198;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -Scalar(8.3700199099999999) * _tmp1 -
      Scalar(9.6622558468725703) * fh1 *
          (-Scalar(1.0) * _tmp0 * _tmp36 * fv1 * std::sinh(_tmp37) -
           Scalar(0.86625939559540499) * _tmp0 -
           (-_tmp0 * _tmp34 +
            Scalar(0.1034955) * _tmp1 * (Scalar(9.6622558468725703) * _tmp31 * _tmp36 - _tmp33)) *
               std::sinh(_tmp35)) +
      Scalar(9.6622558468725703) * std::cosh(_tmp35) -
      Scalar(9.6622558468725703) * std::cosh(_tmp37);
  _res(1, 0) =
      -_tmp157 *
          (-Scalar(0.86565325453551001) * _tmp129 + Scalar(1.0) * _tmp163 * std::sinh(_tmp165) -
           (-Scalar(0.1034955) * _tmp129 * _tmp158 +
            _tmp159 * (-_tmp156 * _tmp164 - _tmp157 * _tmp163)) *
               std::sinh(_tmp160)) -
      _tmp164 * (Scalar(0.86565325453551001) * _tmp155 - std::cosh(_tmp160) + std::cosh(_tmp165));
  _res(2, 0) =
      -_tmp180 *
          (-Scalar(0.87679799772039002) * _tmp184 + Scalar(1.0) * _tmp186 * std::sinh(_tmp189) -
           (-Scalar(0.1034955) * _tmp181 * _tmp184 +
            _tmp187 * (-_tmp179 * _tmp185 - _tmp180 * _tmp186)) *
               std::sinh(_tmp188)) -
      _tmp185 * (Scalar(0.87679799772039002) * _tmp178 - std::cosh(_tmp188) + std::cosh(_tmp189));
  _res(3, 0) =
      -_tmp199 *
          (-Scalar(0.87653584775870996) * _tmp205 + Scalar(1.0) * _tmp207 * std::sinh(_tmp208) -
           (-Scalar(0.1034955) * _tmp200 * _tmp205 +
            _tmp201 * (-_tmp198 * _tmp206 - _tmp199 * _tmp207)) *
               std::sinh(_tmp202)) -
      _tmp206 * (Scalar(0.87653584775870996) * _tmp197 - std::cosh(_tmp202) + std::cosh(_tmp208));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
