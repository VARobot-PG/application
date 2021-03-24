Blockly.Python['setptpcommand'] = function(block) {
  var number_x = block.getFieldValue('x');
  var number_y = block.getFieldValue('y');
  var number_z = block.getFieldValue('z');
  // TODO: Assemble Python into code variable.
  var code = 'SETPTP('+ number_x+','+number_y+','+number_z+')\n';
  return code;
};

Blockly.Python['togglesuctioncommand'] = function(block) {
  var checkbox_suctionenabled = block.getFieldValue('suctionEnabled') == 'TRUE';
  // TODO: Assemble Python into code variable.
  var code = '...\n';
  return code;
};