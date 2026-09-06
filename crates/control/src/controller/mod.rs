use crate::{ControlEffort, ControlState};

pub mod lqr;

pub trait Controller {
    type Error;

    fn compute(&mut self, state: &ControlState) -> Result<ControlEffort, Self::Error>;
}
