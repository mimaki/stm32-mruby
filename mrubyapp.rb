print "Hello, " * 3 + "mruby!\n"
pwm = PWM.new(3)  # D3 pin ��PWM�o�͂Ƃ��Ďg�p
v = 0
d = 1
while true
  pwm.write(v)    # v�̒l���o��
  v += d
  if v == 0
    d = 1
    LED.on        # LED��_��
  elsif v == 255
    d = -1
    LED.off       # LED������
  end
  delay(10)       # 10ms�E�F�C�g
end
