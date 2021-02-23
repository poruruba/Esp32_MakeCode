input.onButtonPressed(Button.A, function () {
  basic.showIcon(IconNames.Heart)
  basic.pause(1000)
  basic.showIcon(IconNames.Happy)
  basic.pause(1000)
  basic.clearScreen()
})
basic.showLeds(`
  . . . . .
  # # . # #
  . . . . .
  . # # # .
  . . . . .
  `)
