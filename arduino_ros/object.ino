void obj()
{ 
  if(digitalRead(pin_objR) == LOW)
  {
    obj_msg.objR = true;
    pub_obj.publish(&obj_msg);
  }
  else
  {
    obj_msg.objR = false;
    pub_obj.publish(&obj_msg);
  }

  if(digitalRead(pin_objL) == LOW)
  {
    obj_msg.objL = true;
    pub_obj.publish(&obj_msg);
  }
  else
  {
    obj_msg.objL = false;
    pub_obj.publish(&obj_msg);
  }
}
