Blockly.Blocks['setptpcommand'] = {
  init: function() {
    this.appendDummyInput()
        .setAlign(Blockly.ALIGN_CENTRE)
        .appendField("MoveTo")
        .appendField("X")
        .appendField(new Blockly.FieldNumber(0, -20, 400), "x")
        .appendField("Y")
        .appendField(new Blockly.FieldNumber(0, -20, 400), "y")
        .appendField("Z")
        .appendField(new Blockly.FieldNumber(0, -20, 400), "z");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("Move Robot to Coordinate (in millimeters)");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['togglesuctioncommand'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("SuctionEnabled")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "suctionEnabled");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("En-/Disable Suction Cup");
 this.setHelpUrl("");
  }
};